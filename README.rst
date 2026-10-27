===========
QEMU README
===========

About this fork (CDEF-inc/qemu, branch ``rx/v11.1``)
====================================================

This is an in-house fork of QEMU 11.1.2 that adds the RXv2, RXv3 and
RXv3e CPU cores to the RX target. It is the test platform of
`rx-rust <https://github.com/CDEF-inc/rx-rust>`_, a Rust / LLVM toolchain
for Renesas RX. The changes are partly AI-generated and are not offered
to upstream QEMU, whose policy (``docs/devel/code-provenance.rst``)
declines such contributions. Everything else in this file is upstream's
README.

What the fork adds to ``qemu-system-rx`` (details in
``docs/system/target-rx.rst``):

* ``-cpu`` models ``rx62n`` (the default), ``rxv1``, ``rxv2``, ``rxv3``
  and ``rxv3e``, with ``fpu``, ``dfpu`` and ``banks`` properties;
* the RXv2 instructions, EXTB, the second accumulator A1 and the RXv2
  DSP instructions;
* the RXv3 instructions (``bfmov``, ``bfmovz``, ``save`` / ``rstr``
  with register banks) and the double-precision FPU (DR0-DR15, DPSW,
  DCMR, DECNT, DEPC and its 21 instructions);
* the CPU side of the RXv3e Protect Zone function;
* fixes found by comparing with real hardware: PSW.U on the switch to
  user mode, the reserved bits of ``mvtipl`` and of the ``mvfac*``
  immediate, ``mvtachi`` and the guard bits, the FPSW flags of an
  underflow with DN = 0, ``ftou`` of negative numbers, the C and Z flags
  of ``suntil`` / ``swhile``, the displacement scale of ``dmov.d`` and
  the DPSW writes of ``mvtdc`` / ``dpopm.l``.

The instruction tests are in rx-rust (``test/qemu``,
``scripts/run-qemu-test.sh``, ``scripts/run-all-tests.sh``). They also run
on an RX72M (RXv3 with the double-precision FPU, on an AP-RX72M-0A board),
and QEMU's ``rxv3`` model gives the same results as the board for every
test that applies to that core; rx-rust's ``docs/NOTES-qemu.md`` lists what differed and how it was
fixed, and ``docs/NOTES-hw.md`` how the board tests are run.

Building only the RX system emulator:

.. code-block:: shell

  git clone https://github.com/CDEF-inc/qemu -b rx/v11.1
  cd qemu && mkdir build && cd build
  ../configure --target-list=rx-softmmu --disable-docs
  ninja qemu-system-rx

rx-rust's test scripts use ``../qemu/build/qemu-system-rx`` when the fork
is checked out next to rx-rust.

The fork is licensed like QEMU (GPL-2.0-or-later for the RX changes).


About QEMU
==========

QEMU is a generic and open source machine & userspace emulator and
virtualizer.

QEMU is capable of emulating a complete machine in software without any
need for hardware virtualization support. By using dynamic translation,
it achieves very good performance. QEMU can also integrate with the Xen
and KVM hypervisors to provide emulated hardware while allowing the
hypervisor to manage the CPU. With hypervisor support, QEMU can achieve
near native performance for CPUs. When QEMU emulates CPUs directly it is
capable of running operating systems made for one machine (e.g. an ARMv7
board) on a different machine (e.g. an x86_64 PC board).

QEMU is also capable of providing userspace API virtualization for Linux
and BSD kernel interfaces. This allows binaries compiled against one
architecture ABI (e.g. the Linux PPC64 ABI) to be run on a host using a
different architecture ABI (e.g. the Linux x86_64 ABI). This does not
involve any hardware emulation, simply CPU and syscall emulation.

QEMU aims to fit into a variety of use cases. It can be invoked directly
by users wishing to have full control over its behaviour and settings.
It also aims to facilitate integration into higher level management
layers, by providing a stable command line interface and monitor API.
It is commonly invoked indirectly via the libvirt library when using
open source applications such as oVirt, OpenStack and virt-manager.

QEMU as a whole is released under the GNU General Public License,
version 2. For full licensing details, consult the LICENSE file.


Documentation
=============

Documentation can be found hosted online at
`<https://www.qemu.org/documentation/>`_. The documentation for the
current development version that is available at
`<https://www.qemu.org/docs/master/>`_ is generated from the ``docs/``
folder in the source tree, and is built by `Sphinx
<https://www.sphinx-doc.org/en/master/>`_.


Building
========

QEMU is multi-platform software intended to be buildable on all modern
Linux platforms, OS-X, Win32 (via the Mingw64 toolchain) and a variety
of other UNIX targets. The simple steps to build QEMU are:


.. code-block:: shell

  mkdir build
  cd build
  ../configure
  make

Additional information can also be found online via the QEMU website:

* `<https://wiki.qemu.org/Hosts/Linux>`_
* `<https://wiki.qemu.org/Hosts/Mac>`_
* `<https://wiki.qemu.org/Hosts/W32>`_


Submitting patches
==================

The QEMU source code is maintained under the GIT version control system.

.. code-block:: shell

   git clone https://gitlab.com/qemu-project/qemu.git

When submitting patches, one common approach is to use 'git
format-patch' and/or 'git send-email' to format & send the mail to the
qemu-devel@nongnu.org mailing list. All patches submitted must contain
a 'Signed-off-by' line from the author. Patches should follow the
guidelines set out in the `style section
<https://www.qemu.org/docs/master/devel/style.html>`_ of
the Developers Guide.

Additional information on submitting patches can be found online via
the QEMU website:

* `<https://wiki.qemu.org/Contribute/SubmitAPatch>`_
* `<https://wiki.qemu.org/Contribute/TrivialPatches>`_

The QEMU website is also maintained under source control.

.. code-block:: shell

  git clone https://gitlab.com/qemu-project/qemu-web.git

* `<https://www.qemu.org/2017/02/04/the-new-qemu-website-is-up/>`_

A 'git-publish' utility was created to make above process less
cumbersome, and is highly recommended for making regular contributions,
or even just for sending consecutive patch series revisions. It also
requires a working 'git send-email' setup, and by default doesn't
automate everything, so you may want to go through the above steps
manually for once.

For installation instructions, please go to:

*  `<https://github.com/stefanha/git-publish>`_

The workflow with 'git-publish' is:

.. code-block:: shell

  $ git checkout master -b my-feature
  $ # work on new commits, add your 'Signed-off-by' lines to each
  $ git publish

Your patch series will be sent and tagged as my-feature-v1 if you need to refer
back to it in the future.

Sending v2:

.. code-block:: shell

  $ git checkout my-feature # same topic branch
  $ # making changes to the commits (using 'git rebase', for example)
  $ git publish

Your patch series will be sent with 'v2' tag in the subject and the git tip
will be tagged as my-feature-v2.

Bug reporting
=============

The QEMU project uses GitLab issues to track bugs. Bugs
found when running code built from QEMU git or upstream released sources
should be reported via:

* `<https://gitlab.com/qemu-project/qemu/-/issues>`_

If using QEMU via an operating system vendor pre-built binary package, it
is preferable to report bugs to the vendor's own bug tracker first. If
the bug is also known to affect latest upstream code, it can also be
reported via GitLab.

For additional information on bug reporting consult:

* `<https://wiki.qemu.org/Contribute/ReportABug>`_


ChangeLog
=========

For version history and release notes, please visit
`<https://wiki.qemu.org/ChangeLog/>`_ or look at the git history for
more detailed information.


Contact
=======

The QEMU community can be contacted in a number of ways, with the two
main methods being email and IRC:

* `<mailto:qemu-devel@nongnu.org>`_
* `<https://lists.nongnu.org/mailman/listinfo/qemu-devel>`_
* #qemu on irc.oftc.net

Information on additional methods of contacting the community can be
found online via the QEMU website:

* `<https://wiki.qemu.org/Contribute/StartHere>`_
