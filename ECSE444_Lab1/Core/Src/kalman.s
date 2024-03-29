	.text
	.global kalman
	// .global update

kalman:
	PUSH {R0-R5}
	VPUSH.F32 {S0-S5}
	MOV R3, R0
	MOV R4, R1
	VLDR.F32 S1, [R3]
	VLDR.F32 S2, [R3,#4]
	VLDR.F32 S3, [R3,#8]
	VLDR.F32 S4, [R3, #12]
	VLDR.F32 S5, [R3, #16]
	MOV R5, #0x0FF
	VMOV.F32 S10, R5
	B update

check:
	B done

update:
	// 1 bit sign, 8 bit exp, 23 BIT MANTISSA
	VADD.F32 S4, S4, S1 // P = P + Q
	VADD.F32 S6, S4, S2 // TEMP = P+R
	VCMP.F32 S6, #0 //if P+R = 0 end
	BEQ check
	VDIV.F32 S5, S4, S6 // K = P/TEMP
	VSUB.F32 S7, S0, S3 // MEASUREMENT - X
	VCMP.F32 S7,  S10// overflow check
	BEQ done
	VMUL.F32 S8, S5, S7 // K * (MEASUREMENT - X)
	VCMP.F32 S8,  S10// overflow check
	BEQ done
	VADD.F32 S3, S3, S8 // X = X + K*(MEASUREMENT - X)
	VCMP.F32 S3,  S10// overflow check
	BEQ done
	VMUL.F32 S9, S4, S5 //PK
	VSUB.F32 S4, S4, S9 // P - PK = P

done:
	// VSTR.F32 S3, [R1]
	VSTR.F32 S1, [R3]
	VSTR.F32 S2, [R3, #4]
	VSTR.F32 S3, [R3, #8]
	VSTR.F32 S4, [R3, #12]
	VSTR.F32 S5, [R3, #16]

	//POP {R0-R5}
	//VPOP.F32 {S0-S5}
	BX LR

	.end


