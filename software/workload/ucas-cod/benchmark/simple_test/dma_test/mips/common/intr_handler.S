.data
last_tail_ptr:
        .word 0

.globl intr_handler
	.align  2
	.type intr_handler, @function
	.section .exception

intr_handler:
	# TODO: Please add your own interrupt handler for DMA engine 

	#base: 0x60020000
	#offset:
	#tail_ptr: 0x0008
	#dma_size: 0x0010
	#ctrl_stat: 0x0014

	#获取tail_ptr的值存到k0寄存器, 计算前后差值
	lui		$k0, 0x6002
	lw		$k0, 0x8 ($k0)
	sub		$k1, $k0, $k1

	#通过循环计算子缓冲区数并更新dam_buf_stat值
LOOP:
	#每次dam_buf_stat减一
	lw		$k0, 0x10($0)
	addi	$k0, $k0, 0xffff
	sw		$k0, 0x10($0)

	#每次减去一个子缓冲区数
	lui		$k0, 0x6002
	lw		$k0, 0x10($k0)
	sub		$k1, $k1, $k0

	bgtz $k1, LOOP

	#将DMA的ctrl_stat寄存器INTR标志位清0
	lui		$k1, 0x6002
	lw		$k0, 0x14 ($k1)
	andi	$k0, $k0, 0x1
	sw		$k0, 0x14 ($k1)

	#记录tail_ptr的值到k1寄存器
	lw		$k1, 0x8 ($k1)

	#退出中断
	eret