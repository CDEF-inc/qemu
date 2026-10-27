.. _RX-System-emulator:

RX System emulator
--------------------

Use the executable ``qemu-system-rx`` to simulate RX target (GDB simulator).
This target emulated following devices.

-  R5F562N8 MCU

   -  On-chip memory (ROM 512KB, RAM 96KB)
   -  Interrupt Control Unit (ICUa)
   -  8Bit Timer x 1CH (TMR0,1)
   -  Compare Match Timer x 2CH (CMT0,1)
   -  Serial Communication Interface x 1CH (SCI0)

-  External memory 16MByte

The core defaults to the RX62N's (RXv1 with the FPU). ``-cpu`` selects
another core on the same board; the peripherals stay those of the RX62N.

-  ``rx62n``: RXv1 with the FPU
-  ``rxv1``: RXv1 without the FPU (RX100, RX200)
-  ``rxv2``: RXv2 with the FPU
-  ``rxv3``: RXv3 with the FPU and the double-precision FPU
-  ``rxv3e``: RXv3e with the FPU and the double-precision FPU

The units can be switched with the ``fpu`` and ``dfpu`` properties, for
example ``-cpu rxv3,dfpu=off``. RXv3 cores have 16 save register banks for
``save`` / ``rstr`` (as the RX72N); ``banks=N`` (0 to 256) changes that,
and ``banks=0`` makes the instructions undefined. Instructions of an ISA or unit the core
does not have raise the undefined instruction exception.

RXv2 and later cores have the exception table register (EXTB), which
moves the fixed vectors (all but reset), and the RXv2 instructions
``movli``/``movco``, ``stz``/``stnz`` with a register source, the
three-operand ``fadd``/``fsub``/``fmul``, ``fsqrt``, ``ftou`` and ``utof``.
RXv2 and later cores also have the second accumulator A1, 72-bit
accumulators and the RXv2 DSP instructions (``emaca``, ``emsba``,
``emula``, ``maclh``, ``msbhi``, ``msblh``, ``msblo``, ``mullh``,
``mvfacgu``, ``mvfaclo``, ``mvtacgu``, ``racl``, ``rdacl``, ``rdacw``, and
the A1 and shift forms of the RXv1 ones).

RXv3 and later cores add ``bfmov``, ``bfmovz``, the three-operand ``xor``
and the register bank save function (``save``, ``rstr``).

With ``dfpu`` (the default on ``rxv3`` and ``rxv3e``), the double-precision
FPU: DR0-DR15, the control registers DPSW, DCMR, DECNT and DEPC, and its 21
instructions. Double-precision floating-point exceptions are reported as an
interrupt request to the interrupt controller; the RX62N based boards have
no source for that request, so it is only recorded in DEPC and DECNT.

Example of ``qemu-system-rx`` usage for RX is shown below:

Download ``<u-boot_image_file>`` from
https://osdn.net/users/ysato/pf/qemu/dl/u-boot.bin.gz

Start emulation of rx-virt::
  qemu-system-rx -M gdbsim-r5f562n8 -bios <u-boot_image_file>

Download ``kernel_image_file`` from
https://osdn.net/users/ysato/pf/qemu/dl/zImage

Download ``device_tree_blob`` from
https://osdn.net/users/ysato/pf/qemu/dl/rx-virt.dtb

Start emulation of rx-virt::
  qemu-system-rx -M gdbsim-r5f562n8 \
      -kernel <kernel_image_file> -dtb <device_tree_blob> \
      -append "earlycon"
