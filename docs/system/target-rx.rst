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
example ``-cpu rxv3,dfpu=off``. Instructions of an ISA or unit the core
does not have raise the undefined instruction exception.

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
