#pragma once
/*
* type_def.h
*
*  Created on: 2017.12.6
*      Author: bwang
*     Summary:	use for Improving the portability of program
*/

#ifndef SERVO_CONTROL_STDKERNEL_INCLUDE_TYPE_DEF_H_
#define SERVO_CONTROL_STDKERNEL_INCLUDE_TYPE_DEF_H_

//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//									BASIC DATA TYPE DEFINITION
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
typedef		char							int8;
typedef		short							int16;
typedef		long							int32;
typedef		long long						int64;

typedef		unsigned char					Uint8;
typedef		unsigned short					Uint16;
typedef		unsigned long					Uint32;
typedef		unsigned long long				Uint64;
//------------------------------------------------------------------------------------------------
typedef		volatile signed char			vint8;
typedef		volatile short					vint16;
typedef		volatile long					vint32;

typedef		volatile unsigned char			vUint8;
typedef		volatile unsigned short			vUint16;
typedef		volatile unsigned long			vUint32;
//------------------------------------------------------------------------------------------------
typedef		const	Uint8					CUint8;
typedef		const	Uint16					CUint16;
typedef		const	Uint32					CUint32;
//------------------------------------------------------------------------------------------------



//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//									BASIC CONSTANT DEFINITION
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
#ifndef		FALSE
#define		FALSE		0
#endif
//------------------------------------------------------------------------------------------------
#ifndef		TRUE
#define		TRUE		1
#endif
//------------------------------------------------------------------------------------------------
#ifndef		BUSY
#define		BUSY		1
#endif
//------------------------------------------------------------------------------------------------
#ifndef		FREE
#define		FREE		0
#endif
//------------------------------------------------------------------------------------------------
#ifndef		WAIT
#define		WAIT		1
#endif
//------------------------------------------------------------------------------------------------
#ifndef		NOWAIT
#define		NOWAIT		0
#endif
//------------------------------------------------------------------------------------------------
#ifndef		ENABLE
#define		ENABLE		1
#endif
//------------------------------------------------------------------------------------------------
#ifndef		DISABLE
#define		DISABLE		0
#endif

//#define		DEBUG_PROCESS
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//									DOUBLE	BYTE DATA TYPE DEFINITION
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
#pragma pack(4)

typedef union
{
	Uint16							w;												// word type
	struct { Uint8 l; Uint8 h; }		b;												// low byte / high byte
																						//------------------------------------------------------------------------------------------------
	int16							ssort;											// signed short type
	Uint16							usort;											// unsigned short type
																					//------------------------------------------------------------------------------------------------
	int8							sch[2];											// signed char type
	Uint8							uch[2];											// unsigned char type
} DBYTEX;

#pragma pack()

//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//									DOUBLE WORD DATA TYPE DEFINITION
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
typedef union
{
	Uint32							dw;												// double word
	struct { Uint16 l; Uint16 h; }	w;												// low word / high word
																					//------------------------------------------------------------------------------------------------
	int32							slg;											// signed long
	Uint32							ulg;											// unsigned long
																					//------------------------------------------------------------------------------------------------
	int16							ssort[2];										// signed short
	Uint16							usort[2];										// unsigned short
																					//------------------------------------------------------------------------------------------------
	int8							sch[4];											// signed char
	Uint8							uch[4];											// unsigned char
} DWORDX;


//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//									BASIC MACRO DEFINITION
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
#define		LowByteX(x)				(((Uint8 *)(&(x)))[0])
#define		HigByteX(x)				(((Uint8 *)(&(x)))[1])
//------------------------------------------------------------------------------------------------
#define		LowWordX(x)				(((Uint16 *)(&(x)))[0])
#define		HigWordX(x)				(((Uint16 *)(&(x)))[1])
//------------------------------------------------------------------------------------------------
#define		LongOfLH(l, h)			((int32)((Uint16)(l) + ((int16)(h) << 16)))
#define		ULongOfLH(l, h)			((Uint32)((Uint16)(l) + ((Uint32)(h) << 16)))
//------------------------------------------------------------------------------------------------



//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//								BIT OPERATION MACRO DEFINITION
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
#define		BITSET(Data, BitNo)		{(Data |= (1 << BitNo));}
#define		BITCLR(Data, BitNo)		{(Data &= ~(1 << BitNo));}
#define		BITCHK(Data, BitNo)		(((Data) & (1 << (BitNo))) != 0)


#endif /* SERVO_CONTROL_STDKERNEL_INCLUDE_TYPE_DEF_H_ */
