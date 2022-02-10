/* 
 * CPU Definations 
 */
#define Irq_Save(_flags)	_flags = _Irq_Save();
static inline void disable_irq() {
   /* __asm volatile("CPSID if"); */
    __disable_irq();
    __disable_fault_irq();
}

static inline unsigned long _Irq_Save(void) {
   /* Get current interrupt (IRQ & FIQ) state to be saved */
   uint32 val = (__get_PRIMASK() << 0) | (__get_FAULTMASK() << 1);
   
   /* Disable interrupts */
   disable_irq();
   
   return val;
}


#define Irq_Restore(_flags)	_Irq_Restore(_flags);
static inline void enable_irq() {
   __asm volatile("CPSIE if");
}

static inline void _Irq_Restore(unsigned mask) {
   __set_PRIMASK(mask >> 0);
   __set_FAULTMASK(mask >> 1);
}

