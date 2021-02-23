/*
 *  Copyright (C) 2002-2013  The DOSBox Team
 *  Copyright (C) 2013-2014  bjt, elianda
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*
 * ------------------------------------------
 * SoftMPU by bjt - Software MPU-401 Emulator
 * ------------------------------------------
 *
 * Based on original midi.c from DOSBox
 *
 */

/* SOFTMPU: Moved exported functions & types to header */
#include "export.h"

/* SOFTMPU: Don't need these includes */
/*#include <assert.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <algorithm>

#include "SDL.h"

#include "dosbox.h"
#include "cross.h"
#include "support.h"
#include "setup.h"
#include "mapper.h"
#include "pic.h"
#include "hardware.h"
#include "timer.h"*/

/* SOFTMPU: Additional defines, typedefs etc. for C */
typedef unsigned long Bit32u;
typedef int Bits;

#define SYSEX_SIZE 1024
#define RAWBUF  1024

/* SOFTMPU: Note tracking for RA-50 */
#define MAX_TRACKED_CHANNELS 16
#define MAX_TRACKED_NOTES 8

#define MAX_CMS_CHANNELS 15

static char* MIDI_welcome_msg = "\xf0\x41\x10\x16\x12\x20\x00\x00    SoftMPU v1.9    \x24\xf7"; /* SOFTMPU */

static Bit8u MIDI_note_off[3] = { 0x80,0x00,0x00 }; /* SOFTMPU */

static Bit8u MIDI_evt_len[256] = {
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x00
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x10
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x20
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x30
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x40
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x50
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x60
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,  // 0x70

  3,3,3,3, 3,3,3,3, 3,3,3,3, 3,3,3,3,  // 0x80
  3,3,3,3, 3,3,3,3, 3,3,3,3, 3,3,3,3,  // 0x90
  3,3,3,3, 3,3,3,3, 3,3,3,3, 3,3,3,3,  // 0xa0
  3,3,3,3, 3,3,3,3, 3,3,3,3, 3,3,3,3,  // 0xb0

  2,2,2,2, 2,2,2,2, 2,2,2,2, 2,2,2,2,  // 0xc0
  2,2,2,2, 2,2,2,2, 2,2,2,2, 2,2,2,2,  // 0xd0

  3,3,3,3, 3,3,3,3, 3,3,3,3, 3,3,3,3,  // 0xe0

  0,2,3,2, 0,0,1,0, 1,0,1,1, 1,0,1,0   // 0xf0
};

static Bit8u CMSFreqMap[128] = {
		0,3,7,11,15,19,23,27,
		31,34,38,41,45,48,51,55,
		58,61,64,66,69,72,75,77,
		80,83,86,88,91,94,96,99,
		102,104,107,109,112,114,116,119,
		121,123,125,128,130,132,134,136,
		138,141,143,145,147,149,151,153,
		155,157,159,161,162,164,166,168,
		170,172,174,175,177,179,181,182,
		184,186,188,189,191,193,194,196,
		197,199,200,202,203,205,206,208,
		209,210,212,213,214,216,217,218,
		219,221,222,223,225,226,227,228,
		229,231,232,233,234,235,236,237,
		239,240,241,242,243,244,245,246,
		247,249,250,251,252,253,254,255
	};

// Volume
static Bit8u atten[128] = {
                 0,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,
                 3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,
                 5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,
                 7,7,7,7,7,7,7,7,8,8,8,8,8,8,8,8,
                 9,9,9,9,9,9,9,9,10,10,10,10,10,10,10,10,
                 11,11,11,11,11,11,11,11,12,12,12,12,12,12,12,12,
                 13,13,13,13,13,13,13,13,14,14,14,14,14,14,14,14,
        	 15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
        };
        	 
// Logic channel - first chip/second chip
static Bit8u ChanReg[15] =  {000,001,002,003,004,005,000,001,002,003,004,005};

// Set octave command
static Bit8u OctavReg[15] = {0x10,0x10,0x11,0x11,0x12,0x12,0x10,0x10,0x11,0x11,0x12,0x12};

/* SOFTMPU: Note tracking for RA-50 */
typedef struct {
        Bit8u used;
        Bit8u next;
        Bit8u notes[MAX_TRACKED_NOTES];
} channel;

channel tracked_channels[MAX_TRACKED_CHANNELS];

static struct {
	Bitu mpuport;
        Bitu sbport;
	Bitu cmsport;
        Bitu serialport;
	Bitu status;
	Bitu cmd_len;
	Bitu cmd_pos;
	Bit8u cmd_buf[8];
	Bit8u rt_buf[8];
	struct {
		Bit8u buf[SYSEX_SIZE];
		Bitu used;
                Bitu usedbufs;
		Bitu delay;
		Bit32u start;
	} sysex;
        bool fakeallnotesoff;
	bool available;
	/*MidiHandler * handler;*/ /* SOFTMPU */
} midi;

typedef struct {
        Bit8u enabled;
        Bit8u note;
        Bit8u volume;
} mid_channel;

mid_channel cms_synth[MAX_CMS_CHANNELS];	// CMS synth

Bitu CmsOctaveStore[11];

Bit8u ChanEnableReg[2] = {0,0};

/* SOFTMPU: Sysex delay is decremented from PIC_Update */
Bitu MIDI_sysex_delay;

/* SOFTMPU: Also used by MPU401_ReadStatus */
OutputMode MIDI_output_mode;

/* SOFTMPU: Initialised in mpu401.c */
extern QEMMInfo qemm;

void _fastcall cmsWrite(void)
{
/*
	parameters
	dx = port base+offset
	ah = register
	al = data
*/
_asm
    {
	inc  dx
	xchg al,ah
                        cmp     qemm.installed,1
                        jne     REGUntrappedOUT
			push 	ax
			push	bx
                        mov     bl,al                   ; bl = value
                        mov     ax,01A01h               ; QPI_UntrappedIOWrite
                        call    qemm.qpi_entry
			pop	bx
			pop	ax
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        REGUntrappedOUT:
	out  dx,al
	dec  dx
	xchg al,ah
                        cmp     qemm.installed,1
                        jne     DATUntrappedOUT
			push	ax
			push	bx
                        mov     bl,al                   ; bl = value
                        mov     ax,01A01h               ; QPI_UntrappedIOWrite
                        call    qemm.qpi_entry
			pop	bx
			pop	ax
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        DATUntrappedOUT:
	out  dx,al

    }
}

void _fastcall cmsNull(void)
{
/*
	parameters
	dx = port offset to null
*/
_asm
    {
	add   dx,220h // FIXME! CMSPortAddr
	mov   cx,20h
	xor   ax,ax
loop_nul:           // null all 20 registers 
	call  cmsWrite
	inc   ah
	loop  loop_nul

	mov   ax,1C02h // reset chip 
	call  cmsWrite

	mov   ax,1C01h // enable this chip 
	call  cmsWrite
    }
}

void _fastcall cmsReset(void)
{
   Bit8u i;
_asm
    {
	mov  dx,0
	call cmsNull
	mov  dx,2
	call cmsNull
    }
	for (i=0;i<11;i++)CmsOctaveStore[i]=0;
	ChanEnableReg[0]=0;
	ChanEnableReg[1]=0;
        for (i=0;i<MAX_CMS_CHANNELS;i++)
        {
                cms_synth[i].enabled=0;
                cms_synth[i].note=0;
                cms_synth[i].volume=0;
        }
}

void cmsNoteOff(Bit8u voice)
{
_asm
   {
		xor   bh,bh
		mov   bl,voice

		mov   dx,220h	// FIXME !!!

		mov   bl,ChanReg[bx]	; bl = true channel (0 - 5)

		xor   di,di
		mov   cl,voice
		cmp   cl,06h
                jl    skip_inc
		inc   di
                add   dx,2
skip_inc:
		mov   al,14h
		inc   dx
                        cmp     qemm.installed,1
                        jne     VRegUntrappedOUT
			push 	bx
                        mov     bl,al                   ; bl = value
                        mov     ax,01A01h               ; QPI_UntrappedIOWrite
                        call    qemm.qpi_entry
			pop	bx
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        VRegUntrappedOUT:
		out   dx,al
		dec   dx

		mov   al,ChanEnableReg[di]
		mov   ah,01h
		mov   cl,bl
		shl   ah,cl
		not   ah
		and   al,ah		; al = voice enable reg

                	cmp     qemm.installed,1
                        jne     ChUntrappedOUT
			push ax
                        mov     bl,al                   ; bl = value
                        mov     ax,01A01h               ; QPI_UntrappedIOWrite
                        call    qemm.qpi_entry
			pop ax
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        ChUntrappedOUT:
		out   dx,al
		mov   ChanEnableReg[di],al
    }
}


void cmsSetVolume(Bit8u voice,Bit8u amplitudeLeft,Bit8u amplitudeRight)
{
_asm
   {
		xor   bh,bh
		mov   bl,voice
		
		mov   dx,220h	// FIXME !!!
		cmp   bl,06h		; check channel num > 5?
		jl    setVol	; yes - set port = port + 2
		add   dx,2
setVol:
		mov   bl,ChanReg[bx]	; bx = true channel (0 - 5)
		mov   al,byte ptr amplitudeLeft
		mov   ah,byte ptr amplitudeRight
		;and   al,0Fh
		mov   cl,4
		shl   ah,cl
		or    al,ah
		mov   ah,bl
		call cmsWrite
   }
}

void cmsSound(Bit8u voice,Bit8u freq,Bit8u octave,Bit8u amplitudeLeft,Bit8u amplitudeRight)
{
_asm
   {
		xor   bh,bh
		mov   bl,voice
		
		mov   dx,220h	// FIXME !!!
		cmp   bl,06h		; check channel num > 5?
		jl    setOctave	; yes - set port = port + 2
		add   dx,2
setOctave:
		mov   bl,ChanReg[bx]	; bx = true channel (0 - 5)
		mov   ah,OctavReg[bx]   ; ah = Set octave command
;
;	ah now = register
;		0,1,6,7=$10
;		2,3,8,9=$11
;		4,5,10,11=$12
;
;	CMS octave regs are write only, so we have to track
;	the values in adjoining voices manually

		mov   al,ah
		xor   ah,ah		; ax = set octave cmd (10h - 12h)
		mov   di,ax		; di = ax
		sub   di,010h		; di = octave cmd - 10h (0..2 index)
		mov   cl,voice
		cmp   cl,06h
                jl    skip_inc
		add   di,3
skip_inc:
		mov   ah,al		; set ah back to octave cmd

		mov   al,byte ptr CmsOctaveStore[di]
		mov   bh,octave
		test  bl,01h
		jnz   shiftOctave
		and   al,0F0h
		jmp   outOctave
shiftOctave:
		and   al,0Fh
		mov   cl,4
		shl   bh,cl
outOctave:
		or    al,bh
		mov   byte ptr CmsOctaveStore[di],al
		call cmsWrite		; set octave to CMS
setAmp:
		mov   al,byte ptr amplitudeLeft
		mov   ah,byte ptr amplitudeRight
		;and   al,0Fh
		mov   cl,4
		shl   ah,cl
		or    al,ah
		mov   ah,bl
		call cmsWrite
setFreq:
		mov   al,byte ptr freq
		or    ah,08h


		call cmsWrite
voiceEnable:
		mov   al,14h
		inc   dx
                        cmp     qemm.installed,1
                        jne     VUntrappedOUT
			push 	bx
                        mov     bl,al                   ; bl = value
                        mov     ax,01A01h               ; QPI_UntrappedIOWrite
                        call    qemm.qpi_entry
			pop	bx
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        VUntrappedOUT:
		out   dx,al
		dec   dx

		xor   di,di
		mov   cl,voice
		cmp   cl,06h
                jl    skip_inc2
		inc   di
skip_inc2:
		mov   al,ChanEnableReg[di]
		mov   ah,01h
		mov   cl,bl
		shl   ah,cl
		or    al,ah
                        cmp     qemm.installed,1
                        jne     ChUntrappedOUT
			push ax
                        mov     bl,al                   ; bl = value
                        mov     ax,01A01h               ; QPI_UntrappedIOWrite
                        call    qemm.qpi_entry
			pop ax
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        ChUntrappedOUT:
		out   dx,al
		mov   ChanEnableReg[di],al
    }
}

/*
void cmsFreq(Bit8u iChannel,Bitu iFreq,Bit8u iLevel)
{
	Bit8u outOctave;
	Bitu outFreq;
	if ((iFreq<32) || (iFreq>7823) || (iLevel=0))
            {
		cmsSound(iChannel,0,0,0,0);
            }
	    else
               {
			outOctave=4;
			outFreq=iFreq;
			while (outFreq<489) {
				outFreq=outFreq*2;
				outOctave--;
			}
			while (outFreq>977) {
				outFreq=outFreq / 2;
				outOctave++;
			}
			cmsSound(					
					iChannel,
					CMSFreqMap[((outFreq-489)*128) / 489],
					outOctave,
					iLevel,iLevel
				);
		}
}

*/
static void PlayMsg_SBMIDI(Bit8u* msg,Bitu len)
{
        /* Output a MIDI message to the hardware using SB-MIDI */
        /* Wait for WBS clear, then output a byte */
	_asm
	{
			mov     bx,msg
			mov     cx,len                  ; Assume len < 2^16
			add     cx,bx                   ; Get end ptr
                        mov     dx,midi.sbport
                        add     dx,0Ch                  ; Select DSP write port
	NextByte:       cmp     bx,cx
			je      End
        WaitWBS:        cmp     qemm.installed,1
                        jne     WaitWBSUntrappedIN
			push	bx
                        mov     ax,01A00h               ; QPI_UntrappedIORead
                        call    qemm.qpi_entry
			mov	al,bl
			pop	bx
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        WaitWBSUntrappedIN:
                        in      al,dx
                        or      al,al
                        js      WaitWBS
                        mov     al,038h                 ; Normal mode MIDI output
                        cmp     qemm.installed,1
                        jne     WaitWBSUntrappedOUT
			push 	bx
                        mov     bl,al                   ; bl = value
                        mov     ax,01A01h               ; QPI_UntrappedIOWrite
                        call    qemm.qpi_entry
			pop	bx
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        WaitWBSUntrappedOUT:
                        out     dx,al
        WaitWBS2:       cmp     qemm.installed,1
                        jne     WaitWBS2UntrappedIN
			push	bx
                        mov     ax,01A00h               ; QPI_UntrappedIORead
                        call    qemm.qpi_entry
			mov	al,bl
			pop	bx
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        WaitWBS2UntrappedIN:
                        in      al,dx
                        or      al,al
                        js      WaitWBS2
			mov     al,[bx]
                        cmp     qemm.installed,1
                        jne     WaitWBS2UntrappedOUT
			push 	bx
                        mov     bl,al                   ; bl = value
                        mov     ax,01A01h               ; QPI_UntrappedIOWrite
                        call    qemm.qpi_entry
			pop	bx
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        WaitWBS2UntrappedOUT:
			out     dx,al
                        inc     bx
			jmp     NextByte

                        ; Nothing more to send
	End:            nop
	}
};

static void PlayMsg_Serial(Bit8u* msg,Bitu len)
{
        /* Output a MIDI message to a serial port */
        /* Wait for transmit register clear, then output a byte */
	_asm
	{
			mov     bx,msg
			mov     cx,len                  ; Assume len < 2^16
			add     cx,bx                   ; Get end ptr
                        mov     dx,midi.serialport
        NextByte:       add     dx,05h                  ; Select line status register
                        cmp     bx,cx
			je      End
        WaitTransmit:   cmp     qemm.installed,1
                        jne     WaitTransmitUntrappedIN
			push	bx
                        mov     ax,01A00h               ; QPI_UntrappedIORead
                        call    qemm.qpi_entry
			mov	al,bl
			pop	bx
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        WaitTransmitUntrappedIN:
                        in      al,dx
                        and     al,040h                 ; Shift register empty?
                        jz      WaitTransmit
                        sub     dx,05h                  ; Select transmit data register
			mov     al,[bx]
                        cmp     qemm.installed,1
                        jne     WaitTransmitUntrappedOUT
			push 	bx
                        mov     bl,al                   ; bl = value
                        mov     ax,01A01h               ; QPI_UntrappedIOWrite
                        call    qemm.qpi_entry
			pop	bx
                        _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                        ; Effectively skips next instruction
        WaitTransmitUntrappedOUT:
			out     dx,al
                        inc     bx
			jmp     NextByte

                        ; Nothing more to send
	End:            nop
	}
};


static void PlayMsg_CMS(Bit8u* msg,Bitu len)
{
  Bit8u noteAdr[] = {5, 32, 60, 85, 110, 132, 153, 173, 192, 210, 227, 243}; // The 12 note-within-an-octave values for the SAA1099, starting at B
  Bit8u command = msg[0];
  Bit8u commandMSB  = command & 0xF0;
  Bit8u midiChannel = command & 0x0F;
  Bit8u octave;
  Bit8u noteVal;
  Bit8u i;
  
  if (commandMSB == 0x80) //Note off
  {
    Bit8u note = msg[1];
    //cmsSound(midiChannel,0,0,0,0);
    cmsNoteOff(midiChannel);
    cms_synth[midiChannel].enabled = 0;

  }
  else if (commandMSB == 0x90) //Note on
  {
    Bit8u note = msg[1];
    Bit8u velo = msg[2];
    
    if (velo != 0)
    {
	note = note+1;

	octave = (note / 12) - 1; //Some fancy math to get the correct octave
  	noteVal = note - ((octave + 1) * 12); //More fancy math to get the correct note

	cmsSound(midiChannel,noteAdr[noteVal],octave,atten[velo],atten[velo]);  	

	cms_synth[midiChannel].enabled = 1;
	cms_synth[midiChannel].note = note-1;
	cms_synth[midiChannel].volume = velo;
	
      //cmsFreq(midiChannel,(440 / 32) * pow(2, ((note - 9) / 12)),atten[velo]);
    }
    else if (velo == 0)
	{
    		cmsNoteOff(midiChannel);
		cms_synth[midiChannel].enabled = 0;
	}
  }
  else if (commandMSB == 0xA0) // Key pressure
  {
    //getSerialByte();
    //getSerialByte();
  }
  else if (commandMSB == 0xB0) // Control change
  {
    Bit8u controller = msg[1];
    Bit8u value = msg[2];
    
    //if (controller == 0x01) setDetune(value);
    if (controller == 0x07) 
	{
		cmsSetVolume(midiChannel,atten[value],atten[value]);
		cms_synth[midiChannel].volume = value;
	}
  }
  else if (commandMSB == 0xC0) // Program change
  {
    //byte program = getSerialByte();
  }
  else if (commandMSB == 0xD0) // Channel pressure
  {
    //byte pressure = getSerialByte();
  }
  else if (commandMSB == 0xE0) // Pitch bend
  {
    //byte pitchBendLSB = getSerialByte();
    //byte pitchBendMSB = getSerialByte();
  }
  
/*
  for (i=0;i<MAX_CMS_CHANNELS;i++) 
    if (cms_synth[i].enabled != 0)
	{
		noteVal = cms_synth[i].volume-1;
		if (noteVal != 0)
			{
				cmsSetVolume(i,atten[noteVal],atten[noteVal]);
				cms_synth[i].volume = noteVal;
			}
		else
			{
    				cmsNoteOff(i);
				cms_synth[i].volume = 0;
				cms_synth[i].enabled = 0;
			}
	}
*/
}

static void PlayMsg(Bit8u* msg,Bitu len)
{
        switch (MIDI_output_mode)
        {
        case M_MPU401:
                /* Output a MIDI message to the hardware */
                /* Wait for DRR clear, then output a byte */
                _asm
                {
                                mov     bx,msg
                                mov     cx,len                  ; Assume len < 2^16
                                add     cx,bx                   ; Get end ptr
                                mov     dx,midi.mpuport
                NextByte:       cmp     bx,cx
                                je      End
                                inc     dx                      ; Set cmd port
                WaitDRR:        cmp     qemm.installed,1
                                jne     WaitDRRUntrappedIN
                                push    bx
                                mov     ax,01A00h               ; QPI_UntrappedIORead
                                call    qemm.qpi_entry
                                mov     al,bl
                                pop     bx
                                _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                                ; Effectively skips next instruction
                WaitDRRUntrappedIN:
                                in      al,dx
                                test    al,040h
                                jnz     WaitDRR
                                dec     dx                      ; Set data port
                                mov     al,[bx]
                                cmp     qemm.installed,1
                                jne     WaitDRRUntrappedOUT
                                push    bx
                                mov     bl,al                   ; bl = value
                                mov     ax,01A01h               ; QPI_UntrappedIOWrite
                                call    qemm.qpi_entry
                                pop     bx
                                _emit   0A8h                    ; Emit test al,(next opcode byte)
                                                                ; Effectively skips next instruction
                WaitDRRUntrappedOUT:
                                out     dx,al
                                inc     bx
                                jmp     NextByte

                                ; Nothing more to send
                End:            nop
                }
                break;
        case M_SBMIDI:
                return PlayMsg_SBMIDI(msg,len);
        case M_SERIAL:
                return PlayMsg_Serial(msg,len);
	case M_CMS:
                return PlayMsg_CMS(msg,len);
        default:
                break;
        }
};

/* SOFTMPU: Fake "All Notes Off" for Roland RA-50 */
static void FakeAllNotesOff(Bitu chan)
{
        Bitu note;
        channel* pChan;

        MIDI_note_off[0] &= 0xf0;
        MIDI_note_off[0] |= (Bit8u)chan;

        pChan=&tracked_channels[chan];

        for (note=0;note<pChan->used;note++)
        {
                MIDI_note_off[1]=pChan->notes[note];
                PlayMsg(MIDI_note_off,3);
        }

        pChan->used=0;
        pChan->next=0;
}

void MIDI_RawOutByte(Bit8u data) {
        channel* pChan; /* SOFTMPU */

        if (midi.sysex.start && MIDI_sysex_delay) {
                _asm
                {
                                ; Bit 4 of port 061h toggles every 15.085us
                                ; Use this to time the remaining sysex delay
                                mov     ax,MIDI_sysex_delay
                                mov     bx,17                   ; Assume 4kHz RTC
                                mul     bx                      ; Convert to ticks, result in ax
                                mov     cx,ax
                                in      al,061h
                                and     al,010h                 ; Get initial value
                                mov     bl,al
                TestPort:       in      al,061h
                                and     al,010h
                                cmp     al,bl
                                je      TestPort                ; Loop until toggled
                                xor     bl,010h                 ; Invert
                                loop    TestPort
                                mov     MIDI_sysex_delay,0      ; Set original delay to zero
                }
                /*Bit32u passed_ticks = GetTicks() - midi.sysex.start;
                if (passed_ticks < midi.sysex.delay) SDL_Delay(midi.sysex.delay - passed_ticks);*/ /* SOFTMPU */
        }

	/* Test for a realtime MIDI message */
	if (data>=0xf8) {
		midi.rt_buf[0]=data;
		PlayMsg(midi.rt_buf,1);
		return;
	}        
	/* Test for a active sysex tranfer */
	if (midi.status==0xf0) {
		if (!(data&0x80)) {
                        /* SOFTMPU: Large sysex support */
                        /*if (midi.sysex.used<(SYSEX_SIZE-1))*/ midi.sysex.buf[midi.sysex.used++] = data;

                        if (midi.sysex.used==SYSEX_SIZE)
                        {
                                PlayMsg(midi.sysex.buf, SYSEX_SIZE);
                                midi.sysex.used = 0;
                                midi.sysex.usedbufs++;
                        }
			return;
		} else {
			midi.sysex.buf[midi.sysex.used++] = 0xf7;

                        if ((midi.sysex.start) && (midi.sysex.usedbufs == 0) && (midi.sysex.used >= 4) && (midi.sysex.used <= 9) && (midi.sysex.buf[1] == 0x41) && (midi.sysex.buf[3] == 0x16)) {
				/*LOG(LOG_ALL,LOG_ERROR)("MIDI:Skipping invalid MT-32 SysEx midi message (too short to contain a checksum)");*/ /* SOFTMPU */
			} else {
				/*LOG(LOG_ALL,LOG_NORMAL)("Play sysex; address:%02X %02X %02X, length:%4d, delay:%3d", midi.sysex.buf[5], midi.sysex.buf[6], midi.sysex.buf[7], midi.sysex.used, midi.sysex.delay);*/
				PlayMsg(midi.sysex.buf, midi.sysex.used); /* SOFTMPU */
				if (midi.sysex.start) {
                                        if (midi.sysex.usedbufs == 0 && midi.sysex.buf[5] == 0x7F) {
                                            /*midi.sysex.delay = 290;*/ /* SOFTMPU */ // All Parameters reset
                                            MIDI_sysex_delay = 290*(RTCFREQ/1000);
                                        } else if (midi.sysex.usedbufs == 0 && midi.sysex.buf[5] == 0x10 && midi.sysex.buf[6] == 0x00 && midi.sysex.buf[7] == 0x04) {
                                            /*midi.sysex.delay = 145;*/ /* SOFTMPU */ // Viking Child
                                            MIDI_sysex_delay = 145*(RTCFREQ/1000);
                                        } else if (midi.sysex.usedbufs == 0 && midi.sysex.buf[5] == 0x10 && midi.sysex.buf[6] == 0x00 && midi.sysex.buf[7] == 0x01) {
                                            /*midi.sysex.delay = 30;*/ /* SOFTMPU */ // Dark Sun 1
                                            MIDI_sysex_delay = 30*(RTCFREQ/1000);
                                        } else MIDI_sysex_delay = ((((midi.sysex.usedbufs*SYSEX_SIZE)+midi.sysex.used)/2)+2)*(RTCFREQ/1000); /*(Bitu)(((float)(midi.sysex.used) * 1.25f) * 1000.0f / 3125.0f) + 2;
                                        midi.sysex.start = GetTicks();*/ /* SOFTMPU */
				}
			}

			/*LOG(LOG_ALL,LOG_NORMAL)("Sysex message size %d",midi.sysex.used);*/ /* SOFTMPU */
			/*if (CaptureState & CAPTURE_MIDI) {
				CAPTURE_AddMidi( true, midi.sysex.used-1, &midi.sysex.buf[1]);
			}*/ /* SOFTMPU */
		}
	}
	if (data&0x80) {
		midi.status=data;
		midi.cmd_pos=0;
		midi.cmd_len=MIDI_evt_len[data];
		if (midi.status==0xf0) {
			midi.sysex.buf[0]=0xf0;
			midi.sysex.used=1;
                        midi.sysex.usedbufs=0;
		}
	}
	if (midi.cmd_len) {
		midi.cmd_buf[midi.cmd_pos++]=data;
		if (midi.cmd_pos >= midi.cmd_len) {
			/*if (CaptureState & CAPTURE_MIDI) {
				CAPTURE_AddMidi(false, midi.cmd_len, midi.cmd_buf);
                        }*/ /* SOFTMPU */

                        if (midi.fakeallnotesoff)
                        {
                                /* SOFTMPU: Test for "Note On" */
                                if ((midi.status&0xf0)==0x90)
                                {
                                        if (midi.cmd_buf[2]>0)
                                        {
                                                pChan=&tracked_channels[midi.status&0x0f];
                                                pChan->notes[pChan->next++]=midi.cmd_buf[1];
                                                if (pChan->next==MAX_TRACKED_NOTES) pChan->next=0;
                                                if (pChan->used<MAX_TRACKED_NOTES) pChan->used++;
                                        }

                                        PlayMsg(midi.cmd_buf,midi.cmd_len);
                                }
                                /* SOFTMPU: Test for "All Notes Off" */
                                else if (((midi.status&0xf0)==0xb0) &&
                                         (midi.cmd_buf[1]>=0x7b) &&
                                         (midi.cmd_buf[1]<=0x7f))
                                {
                                        FakeAllNotesOff(midi.status&0x0f);
                                }
                                else
                                {
                                        PlayMsg(midi.cmd_buf,midi.cmd_len);
                                }
                        }
                        else
                        {
                                PlayMsg(midi.cmd_buf,midi.cmd_len);
                        }
                        midi.cmd_pos=1;         //Use Running status
		}
	}
}

bool MIDI_Available(void)  {
	return midi.available;
}

/* SOFTMPU: Initialisation */
void MIDI_Init(Bitu mpuport,Bitu sbport,Bitu serialport,OutputMode outputmode,bool delaysysex,bool fakeallnotesoff){
        Bitu i; /* SOFTMPU */
	midi.sysex.delay = 0;
	midi.sysex.start = 0;
	MIDI_sysex_delay = 0; /* SOFTMPU */

        if (delaysysex==true)
	{
		midi.sysex.start = 1; /*GetTicks();*/ /* SOFTMPU */
		/*LOG_MSG("MIDI:Using delayed SysEx processing");*/ /* SOFTMPU */
	}
	midi.mpuport=mpuport;
        midi.sbport=sbport;
        midi.serialport=serialport;
	midi.status=0x00;
	midi.cmd_pos=0;
	midi.cmd_len=0;
        midi.fakeallnotesoff=fakeallnotesoff;
        midi.available=true;
        MIDI_output_mode=outputmode;

        /* SOFTMPU: Display welcome message on MT-32 */
//        for (i=0;i<30;i++)
//        {
//                MIDI_RawOutByte(MIDI_welcome_msg[i]);
//        }

	cmsReset();

        /* SOFTMPU: Init note tracking */
        for (i=0;i<MAX_TRACKED_CHANNELS;i++)
        {
                tracked_channels[i].used=0;
                tracked_channels[i].next=0;
        }
}

/* DOSBox initialisation code */
#if 0
class MIDI:public Module_base{
public:
	MIDI(Section* configuration):Module_base(configuration){
		Section_prop * section=static_cast<Section_prop *>(configuration);
		const char * dev=section->Get_string("mididevice");
		std::string fullconf=section->Get_string("midiconfig");
		/* If device = "default" go for first handler that works */
		MidiHandler * handler;
//              MAPPER_AddHandler(MIDI_SaveRawEvent,MK_f8,MMOD1|MMOD2,"caprawmidi","Cap MIDI");
		midi.sysex.delay = 0;
		midi.sysex.start = 0;
		if (fullconf.find("delaysysex") != std::string::npos) {
			midi.sysex.start = GetTicks();
			fullconf.erase(fullconf.find("delaysysex"));
			LOG_MSG("MIDI:Using delayed SysEx processing");
		}
		std::remove(fullconf.begin(), fullconf.end(), ' ');
		const char * conf = fullconf.c_str();
		midi.status=0x00;
		midi.cmd_pos=0;
		midi.cmd_len=0;
		if (!strcasecmp(dev,"default")) goto getdefault;
		handler=handler_list;
		while (handler) {
			if (!strcasecmp(dev,handler->GetName())) {
				if (!handler->Open(conf)) {
					LOG_MSG("MIDI:Can't open device:%s with config:%s.",dev,conf);
					goto getdefault;
				}
				midi.handler=handler;
				midi.available=true;    
				LOG_MSG("MIDI:Opened device:%s",handler->GetName());
				return;
			}
			handler=handler->next;
		}
		LOG_MSG("MIDI:Can't find device:%s, finding default handler.",dev);     
getdefault:     
		handler=handler_list;
		while (handler) {
			if (handler->Open(conf)) {
				midi.available=true;    
				midi.handler=handler;
				LOG_MSG("MIDI:Opened device:%s",handler->GetName());
				return;
			}
			handler=handler->next;
		}
		/* This shouldn't be possible */
	}
	~MIDI(){
		if(midi.available) midi.handler->Close();
		midi.available = false;
		midi.handler = 0;
	}
};


static MIDI* test;
void MIDI_Destroy(Section* /*sec*/){
	delete test;
}
void MIDI_Init(Section * sec) {
	test = new MIDI(sec);
	sec->AddDestroyFunction(&MIDI_Destroy,true);
}
#endif

/* DOSBox MIDI handler code */
#if 0
class MidiHandler;

MidiHandler * handler_list=0;

class MidiHandler {
public:
	MidiHandler() {
		next=handler_list;
		handler_list=this;
	};
	virtual bool Open(const char * /*conf*/) { return true; };
	virtual void Close(void) {};
	virtual void PlayMsg(Bit8u * /*msg*/) {};
	virtual void PlaySysex(Bit8u * /*sysex*/,Bitu /*len*/) {};
	virtual const char * GetName(void) { return "none"; };
	virtual ~MidiHandler() { };
	MidiHandler * next;
};

MidiHandler Midi_none;

/* Include different midi drivers, lowest ones get checked first for default */

#if defined(MACOSX)

#include "midi_coremidi.h"
#include "midi_coreaudio.h"

#elif defined (WIN32)

#include "midi_win32.h"

#else

#include "midi_oss.h"

#endif

#if defined (HAVE_ALSA)

#include "midi_alsa.h"

#endif
#endif /* if 0 */
