/*
 * mov_avg.s
 *
 * Created on: 2/2/2026
 * Author: Hitesh B, Hou Linxin\
 *
 * CG2028 Assignment, Sem 2, AY 2025/26
 * (c) ECE NUS, 2025
 * Write Student 1’s Name here: Kenneth Wong Cun Wi (A0303203A)
 * Write Student 2’s Name here: Joel Ku (A0308792U)
 */

.syntax unified
 .cpu cortex-m4
 .thumb
 .global mov_avg
 .equ N_MAX, 8
 .bss
 .align 4
 .text
 .align 2

/*
 * brief: this moving average filter calculates the average of the most recent N readings from accel_buffer.
 * note: exception handling catch are handled by caller. For example, it is assumed buffer size 1 <= N <= N_MAX and readings are valid values.
 * 
 * inputs:
 * R0 N (buffer size, which will never be 0)
 * R1 accel_buff (pointer to an int buffer containing the most recent N accelerometer readings)
 *
 * Temp reg:
 * R2 current sum of readings thus far
 * R3 Loop counter i
 * R4 current reading from buffer accel_buffer[R1]
 *
 * Outputs:
 * R0 average value of the N readings in the buffer
 */

mov_avg:
 PUSH {r2-r4, lr}       @ push regs to be modified and return addr in LR onto stack

 MOV r2, #0             @ init sum = 0
 MOV r3, #0             @ init i = 0

loop:
 CMP r3, r0             @ compare loop counter i and buffer size N, set condition flags
 BGE end_loop           @ exit when i >= N  (negation of while loop cond. i < N)

 LDR r4, [r1], #4       @ post index to load next value from buffer into R4 and increment buffer pointer
 ADD r2, r2, r4         @ add value to current sum thus far
 ADD r3, r3, #1         @ increment loop counter i++
 B loop                 @ branch back to loop for next index in buffer

end_loop:
 SDIV r0, r2, r0        @ average = sum / N, store result in return reg R0 to return to caller 
 POP {r2-r4, pc}        @ pop regs to restore their original values and the return addr in LR into PC to return