

// flash layout  ddvs først opcode så r0->r3 og så¨ en mem ref.
// dvs en insytr fylder altid 6*sizeof(int) i dette tilfælde på arduino 12 bytes

struct instruction {
  int opcode;
  int r0, r1, r2, r3;
  int mem;
};

//CPU
int pc;
int regs[4];
struct instruction actOpcode;


// RAM
int ram[] = {2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};


/*
   SUB a,b,c   subtracts reg c fomr reg b and store result in reg a
   example  SUB r0,r2,r3
   ADD a,b,c   ditto
   LD a,m   loads content of memory ay address m to reg a
   example   LD r1,10   load with content of memory address 10
*/
#define SUB 0x00
#define ADD  0x01
#define LD  0x02
#define ST   0x03
#define JMP 0x04
#define LDA  0x05
#define JMPR 0x06
#define AND 0x07
#define OR 0x08
#define DUP 0x09

// lidt smart macro cirkus

// dst: destination
// src: source

#define ld(r,m)           {LD,r,-1,-1,-1,m}
#define lda(r,m)           {LDA,r,-1,-1,-1,m}
#define  st(m,r)          {ST,r,-1,-1,-1,m}
#define jmp(mRel)         {JMP,-1,-1,-1,-1,mRel}

#define add(dstR,r1,r2)   {ADD,dstR,r1,r2,-1,0}
#define sub(dstR,r1,r2)   {SUB,dstR,r1,r2,-1,0}
#define dup(dstR,srcR)    {DUP,dstR,srcR,-1,-1,0}

/*
  struct instruction fl[] = {
  {LD, 0, -1, -1, -1, 10},  // 0   LD R0, @10       -1 indicates not used in this instruction load r0 with content of mem address 10
  {LD, 1, -1, -1, -1, 2},   // 1   LD R1, @2        load r1 with content of mem address 2
  {ADD, 2, 0, 1, -1, 0},    // 2   ADD R2, R0, R1   add r0 and r1 and save in r2
  {ST, 2, -1, -1, -1, 10},   // 3   ST R2, @2        store r2 at mem address 10
  {JMP, -1, -1, -1, -1, -4} // 4   JMP -4           jump rel -4 dvs til addr 0
  };

*/


// nu smartere kodning af flash - macroer expander til det flash ovenfor
struct instruction fl[] = {
  ld(0, 10),    // load r0 med mem på adr 0 (RAM)
  ld(1, 2),     // load r1 med   mem 2
  add(2, 0, 1), // add r0 og r1 og gem  i r2
  st(10, 2),    // gem r2 på mem adr 10
  lda(3, 2),    // load r3 med tallet 2
  add(0, 2, 3),
  ld(1, 3),     // load mem3 i reg 1, læg sammen med reg0 og gem i reg2 og gem reg2 på mem 3
  add(2, 0, 1),
  st(3, 2),
  dup(1, 3),    // dupliker reg3 i reg1
  add(2, 3, 1),
  jmp(-11)      // jmp rel -11 dvs til første instruction
};
 

void dumpCPU()
{
  Serial.print("pc ");
  Serial.print(pc); Serial.print(" - ");
  switch (actOpcode.opcode) {
    case ADD: Serial.print("ADD  ");
      break;
    case SUB: Serial.print("SUB  ");
      break;
    case LD:  Serial.print("LD   ");
      break;
    case ST: Serial.print("ST   ");
      break;
    case JMP: Serial.print("JMP  ");
      break;
    case LDA: Serial.print("LDA  ");
      break;
    case DUP: Serial.print("DUP  ");
      break;
    case JMPR: Serial.print("JMPR ");
      break;
    default: Serial.print(" HM   "); Serial.println(actOpcode.opcode);
  }
  Serial.print(" opcode - regs ");
  Serial.print(actOpcode.r0); Serial.print(" ");
  Serial.print(actOpcode.r1); Serial.print(" ");
  Serial.print(actOpcode.r2); Serial.print(" ");
  Serial.print(actOpcode.r3);

  Serial.print("\t Mem ");
  Serial.print(actOpcode.mem);
  Serial.print(" --\t ");
}

void dumpRegs()
{
   
  Serial.print(" pc "); Serial.print(pc);
  Serial.print(" regs ");

  for (int i = 0 ; i < 4 ; i++) {
    Serial.print(regs[i]); Serial.print(" ");
  }
  Serial.print("-- ");
}


void dumpProg(struct instruction *code, int nrInstr)
{
  int lNr = 0;
  while (nrInstr--) {
    Serial.print("lnr "); Serial.print(lNr); Serial.print("\t");
    switch (code->opcode) {
      case ADD: Serial.print("ADD  ");
        break;
      case SUB: Serial.print("SUB  ");
        break;
      case LD:  Serial.print("LD   ");
        break;
      case ST: Serial.print("ST   ");
        break;
      case JMP: Serial.print("JMP  ");
        break;
      case LDA: Serial.print("LDA  ");
        break;
      case DUP: Serial.print("DUP  ");
        break;
      case JMPR: Serial.print("JMPR ");
        break;
      default: Serial.print(" HM   "); Serial.println(code->opcode);
    }
    Serial.print("\t regs ");
    Serial.print(code->r0); Serial.print(" \t");
    Serial.print(code->r1); Serial.print(" \t");
    Serial.print(code->r2); Serial.print(" \t");
    Serial.print(code->r3);

    Serial.print("\t Mem ");
    Serial.println(code->mem);
    lNr++; code++;
  }
}

void dumpRam()
{
  for (int i = 0; i < sizeof(ram)/sizeof(int); i++) {
    Serial.print(ram[i],HEX); Serial.print(" ");
  }
}

int fetchInstr()
{
  // fetch next instruction
  actOpcode = fl[pc];
}

int doInstr()
{
  switch (actOpcode.opcode) {
    case ADD: regs[actOpcode.r0] = regs[actOpcode.r1]  +  regs[actOpcode.r2];
      pc++;
      break;
    case SUB: regs[actOpcode.r0] = regs[actOpcode.r1]  -  regs[actOpcode.r2];
      pc++;
      break;
    case AND: regs[actOpcode.r0] = regs[actOpcode.r1]  &  regs[actOpcode.r2];
      pc++;
      break;
    case OR:  regs[actOpcode.r0] = regs[actOpcode.r1]  |  regs[actOpcode.r2];
      pc++;
      break;
    case LD:  // LD
      regs[actOpcode.r0] = ram[actOpcode.mem];
      pc++;
      break;
    case LDA:
      regs[actOpcode.r0] = actOpcode.mem;
      pc++;
      break;
    case DUP:
      regs[actOpcode.r0] = regs[actOpcode.r1];
      pc++;
      break;
    case ST:
      ram[actOpcode.mem] = regs[actOpcode.r0];
      pc++;
      break;
    case JMP:
      pc += actOpcode.mem;
      break;
    case JMPR: pc += regs[0];
      break;
    default: Serial.print("HM"); dumpCPU();
  }

}


void setup() {
  Serial.begin(9600);

  Serial.println("a small stupid cpu...");
  dumpProg(fl, sizeof(fl) / sizeof(struct instruction));

  Serial.println("");
}

void loop() {
  fetchInstr();
  dumpCPU();
  doInstr();
  Serial.print("\tafter instr ");
  dumpRegs();
  //dumpRegs();
  dumpRam();
  Serial.println("");
  delay(500);
}
