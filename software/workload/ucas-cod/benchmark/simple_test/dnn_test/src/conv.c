#include "printf.h"
#include "trap.h"
#include "mul.h"
#include "div.h"
#include "perf_cnt.h"

#define FRAC_BIT 10

#define RD_ADDR 135106448
#define RD_SIZE_D0 1
#define RD_SIZE_D1 1
#define RD_SIZE_D2 28
#define RD_SIZE_D3 28

#define WEIGHT_ADDR 134217728
#define WEIGHT_SIZE_D0 20
#define WEIGHT_SIZE_D1 1
#define WEIGHT_SIZE_D2 5
#define WEIGHT_SIZE_D3 5

#define WR_ADDR 135108240
#define WR_SIZE_D0 1
#define WR_SIZE_D1 20
#define WR_SIZE_D2 12
#define WR_SIZE_D3 12

#define KERN_ATTR_CONV_PAD 0
#define KERN_ATTR_CONV_STRIDE 1
#define KERN_ATTR_POOL_PAD 0
#define KERN_ATTR_POOL_KERN_SIZE 2
#define KERN_ATTR_POOL_STRIDE 2

//MMIO register address of DNN accelerator
#define GPIO_START_ADDR    0x60030000
#define GPIO_DONE_ADDR     0x60030008

struct size_vec4
{
	unsigned d0;
	unsigned d1;
	unsigned d2;
	unsigned d3;
};

struct mem_addr
{
	unsigned rd_addr;
	unsigned weight_addr;
	unsigned wr_addr;
};

int mul(short a, short b)
{
#ifndef USE_MUL
	int ans = mul_ll(a, b);
#else
	int ans = a * b;
#endif
	return ans;
}

struct mem_addr addr = {RD_ADDR, WEIGHT_ADDR, WR_ADDR};
struct size_vec4 rd_size = {RD_SIZE_D0, RD_SIZE_D1, RD_SIZE_D2, RD_SIZE_D3};
struct size_vec4 wr_size = {WR_SIZE_D0, WR_SIZE_D1, WR_SIZE_D2, WR_SIZE_D3};
struct size_vec4 weight_size = {WEIGHT_SIZE_D0, WEIGHT_SIZE_D1, WEIGHT_SIZE_D2, WEIGHT_SIZE_D3};

struct size_vec4 conv_size;

extern char _binary_data_result_bin_start[];
extern char _binary_data_result_bin_size[];

void convolution()
{
	short *in = (short *)addr.rd_addr;
	short *weight = (short *)addr.weight_addr;
	short *out = (short *)addr.wr_addr;

	//unsigned output_offset = 0;
	//unsigned input_offset = 0;

	unsigned input_fm_w = rd_size.d3;
	unsigned input_fm_h = rd_size.d2;

	unsigned pad = KERN_ATTR_CONV_PAD;
	unsigned pad_len = pad << 1;

	unsigned conv_out_w = rd_size.d3 - weight_size.d3 + pad_len;
	unsigned conv_out_h = rd_size.d2 - weight_size.d2 + pad_len;

	unsigned stride = KERN_ATTR_CONV_STRIDE;

	conv_out_w = div(conv_out_w, stride);
	conv_out_h = div(conv_out_h, stride);

	conv_out_w++;
	conv_out_h++;

	conv_size.d0 = wr_size.d0;
	conv_size.d1 = wr_size.d1;	//输出特征图通道数
	conv_size.d2 = conv_out_h;	//输出特征图高度
	conv_size.d3 = conv_out_w;	//输出特征图宽度

	//TODO: Please add your implementation here
	int out_position = 0;	//待填入out数组元素的位置
	int single_weight_size = 1 + mul(weight_size.d2, weight_size.d3);	//一个卷积核的大小
	int total_weight_size = mul(rd_size.d1, single_weight_size);	//一次处理所有通道输入的卷积核的大小
	int input_size = mul(input_fm_h, input_fm_w);	//一个输入数组的大小

	for (int no = 0; no < conv_size.d1; no++)	//输出特征图数量
	{
		int bias_position = mul(no, total_weight_size);	//各权重图中bias的位置
		for (int ni = 0; ni < rd_size.d1; ni++)	//输入特征图数量
		{
			int read_input_position = mul(ni, input_size);
			int read_weight_position = mul(ni, single_weight_size);
			for (int y = 0; y < conv_size.d2; y++)	//输出特征图高度
			{
				int sy = mul(y, stride);
				for (int x = 0; x < conv_size.d3; x++)	//输出特征图宽度
				{
					int sx = mul(x, stride);
					out[out_position] = weight[bias_position];
					int out_tem = 0;
					for (int ky = 0; ky < weight_size.d2; ky++)	//权重值高度
						for (int kx = 0; kx < weight_size.d3; kx++)	//权重值宽度
						{
							int iw = kx + sx - pad;
							int ih = ky + sy - pad;
							if (iw < 0 || ih < 0 || iw >= input_fm_w || ih >= input_fm_h)	//边界用0填充
								continue;
							int input_position = read_input_position + mul(ih, input_fm_w) + iw;	//所需数据在input数组中的位置
							int weight_position = bias_position + 1 + read_weight_position + mul(ky, weight_size.d3) + kx;	//所需数据在weight数组中的位置
							out_tem += mul(in[input_position], weight[weight_position]);
						}
					out[out_position] += ((short)(out_tem >> FRAC_BIT) & 0x7fff) | ((short)((out_tem & 0x80000000) >> 16));
					out_position++;
				}
			}
		}
	}
}

void pooling()
{
	short *out = (short *)addr.wr_addr;

	//unsigned output_offset = 0;
	//unsigned input_offset = 0;

	unsigned input_fm_w = conv_size.d3;
	unsigned input_fm_h = conv_size.d2;

	unsigned pad = KERN_ATTR_POOL_PAD;
	unsigned pad_len = pad << 1;

	unsigned pad_w_test = conv_size.d3 - KERN_ATTR_POOL_KERN_SIZE;
	unsigned pad_h_test = conv_size.d2 - KERN_ATTR_POOL_KERN_SIZE;

	unsigned pool_out_w = pad_w_test + pad_len;
	unsigned pool_out_h = pad_h_test + pad_len;

	unsigned stride = KERN_ATTR_POOL_STRIDE;

	unsigned pad_w_test_remain = pad_w_test - mul(div(pad_w_test, stride), stride);
	unsigned pad_h_test_remain = pad_h_test - mul(div(pad_h_test, stride), stride);

	pool_out_w = div(pool_out_w, stride);
	pool_out_h = div(pool_out_h, stride);
	pool_out_w++;
	pool_out_h++;

	if ((!pad) && (pad_w_test_remain || pad_h_test_remain))
	{
		pool_out_w++;
		pool_out_h++;
	}

	//TODO: Please add your implementation here
	int input_size = mul(input_fm_h, input_fm_w);
	int pool_size = mul(pool_out_h, pool_out_w);

	for (int no = 0; no < conv_size.d1; no++)
		for (int y = 0; y < pool_out_h; y++)
			for (int x = 0; x < pool_out_w; x++)	//输出图像尺寸
			{
				short max = 0x8000;	//定点表示最小的数
				for (int ky = 0; ky < KERN_ATTR_POOL_KERN_SIZE; ky++)
					for (int kx = 0; kx < KERN_ATTR_POOL_KERN_SIZE; kx++)	//采样区域尺寸
					{
						int iw = kx + mul(x, stride) - pad;
						int ih = ky + mul(y, stride) - pad;
						short tem;
						if (iw < 0 || ih < 0 || iw >= input_fm_w || ih >= input_fm_h)	//边界用0填充
							tem = 0;
						else
						{
							int out_read_position = mul(no, input_size) + mul(ih, input_fm_w) + iw;
							tem = out[out_read_position];
						}
						if (tem > max)
							max = tem;
					}
				int out_write_position = mul(no, pool_size) + mul(y, pool_out_w) + x;
				out[out_write_position] = max;
			}	
}

#ifdef USE_HW_ACCEL
void launch_hw_accel()
{
	volatile int* gpio_start = (void*)(GPIO_START_ADDR);
	volatile int* gpio_done = (void*)(GPIO_DONE_ADDR);

	//TODO: Please add your implementation here
	*gpio_start = (*gpio_start) | 0x1;
	while(!(*gpio_done) & 0x1);
}
#endif

int comparing()
{
	char *out = (char *)addr.wr_addr;
	char *result = (char *)_binary_data_result_bin_start;

#ifdef USE_HW_ACCEL
	int count = (int)_binary_data_result_bin_size + 
		    (16 - WR_SIZE_D3) * 2 * WR_SIZE_D2 * WR_SIZE_D1;
#else
	int count = (int)_binary_data_result_bin_size;
#endif

	for (int i = 0, j = 0; i < count; i++)
	{
#ifdef USE_HW_ACCEL
		int alignment = i & 0x0000001f;
		if (alignment >= (WR_SIZE_D3 << 1))
			continue;
#endif
		if (*(out + i) != *(result + j))
		{
			printf("Failed! at address %x and %x with data %x and %x\n", out + i, result + j, *(out + i), *(result + j));
			return 1;
		}
		j++;
	}

	printf("Passed!\n");
	return 0;
}

int main()
{
	Result res;
	res.msec = 0;
	bench_prepare(&res);

#ifdef USE_HW_ACCEL
	printf("Launching task...\n");
	launch_hw_accel();
#else
	printf("starting convolution\n");
	convolution();
	printf("starting pooling\n");
	pooling();
#endif

	int result = comparing();

	bench_done(&res);
	if(res.msec_overflow)
		printf("Cycles: %u%09u\n", res.msec_overflow, res.msec);
	else
		printf("Cycles: %u\n", res.msec);
	printf("Memory Load/Store: %u\n", res.mem_c);
    printf("Instructions: %u\n", res.inst_c);
    printf("Cycles Waiting totally: %u\n", res.wait_c);
    printf("Load Instructions: %u\n", res.ld_c);
    printf("Cycles Waiting for Fetching Instructions: %u\n", res.if_c);
    printf("Cycles Waiting for Instructions: %u\n", res.iw_c);
    printf("Cycles Waiting for Requesting Data: %u\n", res.memw_c);
    printf("Cycles Waiting for Reading Data: %u\n", res.rdw_c);
	printf("benchmark finished\n");

	if (result == 0) {
		hit_good_trap();
	} else {
		nemu_assert(0);
	}

	return 0;
}
