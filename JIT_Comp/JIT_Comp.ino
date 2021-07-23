#include "instruction.h"
uint16_t instructionNow = 0;
uint8_t reg0 = 0;
uint8_t reg1;
uint8_t reg2;
uint8_t reg3;

uint8_t ramBank[1024];

uint8_t instructionSet[] = {
  LPM, REG1, 22, LPM, REG2, 33, ADD, JMP, 0x00, 0x00
};

int main(void) {
  for (;;) {
    switch (getIns()) {
      case TST: regSet(1, 0 != getIns()); break;
      case CP: regComp(getIns(), getIns()); break;
      case LPM: regSet(getIns(), getIns()); break;
      case LRM: regSet(getIns(), ramRead((uint16_t)getIns() | (uint16_t)getIns() << 8)); break;
      case SPM: ramSet((uint16_t)getIns() | (uint16_t)getIns() << 8, getIns()); break;
      case SRM: ramSet((uint16_t)getIns() | (uint16_t)getIns() << 8, regRead(getIns())); break;
      case JMP: instructionNow = (uint16_t)getIns() | (uint16_t)getIns() << 8; break;
      case IN: regSet(getIns(), ioRegRead(getIns())); break;
      case OUT: ioRegSet(getIns(), regRead(getIns())); break;
      case ADD: reg1 = reg1 + reg2; reg0 = SREG; break;
      case SUB: reg1 = reg1 - reg2; reg0 = SREG;  break;
      case INCR: reg1++; reg0 = SREG;  break;
      case DECR: reg1--; reg0 = SREG;  break;
      case DIV: reg1 = reg1 / reg2; reg0 = SREG;  break;
      case DIVS: reg1 = (int8_t)reg1 / (int8_t)reg2; reg0 = SREG;  break;
      case MUL: reg1 = reg1 * reg2; reg0 = SREG;  break;
      case MULS: reg1 = (int8_t)reg1 * (int8_t)reg2; reg0 = SREG;  break;
    }
  }
  return 0;
}

inline uint8_t getIns(void) {
  return instructionSet[instructionNow++];
}

inline void regComp(uint8_t regLeft, uint8_t regRight) {
  reg0 &= ~(0x01 << Z | 0x01 << C);
  reg0 |= (regLeft == regRight) << Z;
  reg0 |= (regLeft > regRight) << C;
}

inline void regSet(uint8_t reg, uint8_t data) {
  switch (reg) {
    case REG0: reg0 = data; break;
    case REG1: reg1 = data; break;
    case REG2: reg2 = data; break;
    case REG3: reg3 = data; break;
  }
}

inline uint8_t regRead(uint8_t reg) {
  switch (reg) {
    case REG0: return reg0;
    case REG1: return reg1;
    case REG2: return reg2;
    case REG3: return reg3;
  }
  return 0;
}

inline void ramSet(uint16_t reg, uint8_t data) {
  ramBank[reg] = data;
}

inline uint8_t ramRead(uint16_t reg) {
  return ramBank[reg];
}

inline void ioRegSet(uint8_t reg, uint8_t data) {
  switch (reg) {
    case DDR_D: DDRD = data; break;
    case PORT_D: PORTD = data; break;
    case PIN_D: PIND = data; break;
    case DDR_B: DDRB = data; break;
    case PORT_B: PORTB = data; break;
    case PIN_B: PINB = data; break;
    case DDR_C: DDRC = data; break;
    case PORT_C: PORTC = data; break;
    case PIN_C: PINC = data; break;
  }
}

inline uint8_t ioRegRead(uint8_t reg) {
  switch (reg) {
    case DDR_D: return DDRD;
    case PORT_D: return PORTD;
    case PIN_D: return PIND;
    case DDR_B: return DDRB;
    case PORT_B: return PORTB;
    case PIN_B: return PINB;
    case DDR_C: return DDRC;
    case PORT_C: return PORTC;
    case PIN_C: return PINC;
  }
  return 0;
}
