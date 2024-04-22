FM23 kernel modules
===================
This repository contains a number of kernel modules which are intended
to be used in the FM23 machine Linux kernel.

These modules are intended to be compiled as out-of-tree drivers, using
the Makefile from a full Linux source tree. For the FM23, this happens
as part of the image customization process using
[torizoncore-builder](https://github.com/toradex/torizoncore-builder/)
and can be configured by adding something like this to `tcbuild.yaml`:


```yaml
  kernel:
    modules:
      - source-dir: FM23-kernel-modules/staging-imx
        autoload: no
      - source-dir: FM23-kernel-modules/imx708
        autoload: no
```

License
-------
These modules are taken from various Linux kernel versions and are
licensed under the GPL 2.0 license. The SPDX identifer is:

> SPDX-License-Identifier: GPL-2.0

See the [LICENSE](LICENSE) file to see the full GPL license.
