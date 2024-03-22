.syntax unified
.thumb
.cpu cortex-m0plus
.type division_asm, %function
.global division_asm

division_asm:
    mov r4, r0         @ Guarda el dividendo en r4
    mov r3, r1         @ Carga el divisor en r3
    movs r2, #0        @ Inicializa el contador en r2 a 0
    
loop:
    cmp r4, r3         @ Compara el dividendo con el divisor
    blt end_loop       @ Si el dividendo es menor que el divisor, termina el bucle
    subs r4, r4, r3    @ Resta el divisor del dividendo 
    adds r2, r2, #1    @ Incrementa el contador
    b loop             @ Vuelve al inicio del bucle
    
end_loop:
    movs r0, r2        @ Carga el contador en r0
    bx lr              @ Retorna
    
.end
