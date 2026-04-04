.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.section .isr_vector,"a",%progbits
.type g_pfnVectors, %object
.global g_pfnVectors
g_pfnVectors:
    .word _estack
    .word Reset_Handler
    .word NMI_Handler
    .word HardFault_Handler
    .word MemManage_Handler
    .word BusFault_Handler
    .word UsageFault_Handler
    .word 0
    .word 0
    .word 0
    .word 0
    .word SVCall_Handler
    .word DebugMon_Handler
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler
    .rept 82
    .word Default_Handler
    .endr
.size g_pfnVectors, .-g_pfnVectors

.section .text.Default_Handler,"ax",%progbits
Default_Handler:
    b Default_Handler
.size Default_Handler, .-Default_Handler

.macro def_irq handler
    .weak \handler
    .thumb_set \handler, Default_Handler
.endm

def_irq NMI_Handler
def_irq HardFault_Handler
def_irq MemManage_Handler
def_irq BusFault_Handler
def_irq UsageFault_Handler
def_irq SVCall_Handler
def_irq DebugMon_Handler
def_irq PendSV_Handler
def_irq SysTick_Handler

.section .text.Reset_Handler,"ax",%progbits
.global Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
    bl SystemInit

    /* .data: flash -> RAM */
    ldr r0, =_sidata
    ldr r1, =_sdata
    ldr r2, =_edata
    subs r2, r2, r1
    ble 2f
1:
    ldr r3, [r0], #4
    str r3, [r1], #4
    subs r2, r2, #4
    bgt 1b
2:
    /* .bss clear */
    movs r0, #0
    ldr r1, =_sbss
    ldr r2, =_ebss
    subs r2, r2, r1
    ble 4f
3:
    str r0, [r1], #4
    subs r2, r2, #4
    bgt 3b
4:
    bl __libc_init_array
    bl main
    b .

.size Reset_Handler, .-Reset_Handler
