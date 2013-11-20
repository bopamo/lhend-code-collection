;------------------------------------------------------------------------------
;
; Copyright (c) 2013, Leo Hendrawan
; All rights reserved.
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions are met:
;    * Redistributions of source code must retain the above copyright
;      notice, this list of conditions and the following disclaimer.
;    * Redistributions in binary form must reproduce the above copyright
;      notice, this list of conditions and the following disclaimer in the
;      documentation and/or other materials provided with the distribution.
;    * Neither the name of the copyright holder(s) nor the names of its
;      contributor(s) may be used to endorse or promote products derived
;      from this software without specific prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
; THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
; PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTOR(S) BE LIABLE FOR
; ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
; DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
; SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
; CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
; OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
; USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
;------------------------------------------------------------------------------

;------------------------------------------------------------------------------
;
; File: vector.asm
;
; Description : implementation of secondary vector table for application
;               with OpenBSL implementation
;
;------------------------------------------------------------------------------

    .cdecls C,LIST,"OpenBSL_Device.h"


    .ref TimerA_ISR

    ; make linker not to optimized out the ".app_vector_tbl" section
    ; - only works with EABI binary output file format
    .retain ".app_vector_tbl"

    ; start of the ".vector_tbl" section
    .sect ".app_vector_tbl"

; DEV_VECTOR_INT_IDX_P1             (0)
P1_VECTOR
      NOP         ; empty vector is represented by these two instructions
      RETI


; DEV_VECTOR_INT_IDX_P2             (1)
P2_VECTOR
      NOP         ; empty vector is represented by these two instructions
      RETI

; DEV_VECTOR_INT_IDX_ADC10          (2)
ADC10_VECTOR
      NOP         ; empty vector is represented by these two instructions
      RETI

; DEV_VECTOR_INT_IDX_USCI_RX_TX     (3)
USCI_RX_TX_VECTOR
      NOP         ; empty vector is represented by these two instructions
      RETI


; DEV_VECTOR_INT_IDX_USCI_STAT      (4)
USCI_STAT_VECTOR
      NOP         ; empty vector is represented by these two instructions
      RETI

; DEV_VECTOR_INT_IDX_TA0_1          (5)
TA0_1_VECTOR
      NOP         ; empty vector is represented by these two instructions
      RETI

; DEV_VECTOR_INT_IDX_TA0_0          (6)
TA0_0_VECTOR
      br  #TimerA_ISR

; DEV_VECTOR_INT_IDX_WDT            (7)
WDT_VECTOR
      NOP         ; empty vector is represented by these two instructions
      RETI


; DEV_VECTOR_INT_IDX_COMP_A         (8)
COMP_A_VECTOR
      NOP         ; empty vector is represented by these two instructions
      RETI


; DEV_VECTOR_INT_IDX_TA1_1          (9)
TA1_1_VECTOR
      NOP         ; empty vector is represented by these two instructions
      RETI


; DEV_VECTOR_INT_IDX_TA1_0          (10)
TA1_0_VECTOR
      br  #TimerA_ISR


; DEV_VECTOR_INT_IDX_NMI            (11)
NMI_VECTOR
      NOP         ; empty vector is represented by these two instructions
      RETI






