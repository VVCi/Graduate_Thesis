#ifndef __SYS_IRQ_H__
#define __SYS_IRQ_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define SYS_IRQ_EXCEPTION_NUMBER_IRQ0_NUMBER_RESPECTIVE		16	/* exception number 16 ~~ IRQ0 */

extern void sys_irq_nrf24l01();
extern void sys_irq_shell();
extern void sys_irq_ir_io_rev();
extern void sys_irq_timer_50us();
extern void sys_irq_timer_10ms();
extern void sys_irq_timer_hs1101();
extern void SDIO_IRQHandler(void);
extern void DMA2_Stream3_IRQHandler(void);
extern void spi2_irq(void);
extern void uart2_irq(void);
extern void sys_irq_uart_cpu_serial_if();
#ifdef __cplusplus
}
#endif

#endif // __SYS_IRQ_H__
