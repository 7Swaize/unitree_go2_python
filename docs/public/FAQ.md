## Receiving `PublishSubscribeOpenError(UnableToOpenDynamicServiceInformation)` Error

There are stale resources that need to be cleaned up. Usually this is done automatically via Icoryx2 internal.
However, if cleanup didn't happen automatically, we must remove stale resources.

We provided a script to do that for you.

Linux:

```bash
bash ./go2-control/scripts/unix/clear_iceoryx_caches.sh
``` 

Windows:

Execute the `go2-control/scripts/windows/clear_iceoryx_caches.bat` script. You may need to run it as administrator.


## Can I create multiple `Go2Controller` instances? 

Maybe. Currently, no guarantees are made about using multiple `Go2Controller` instances within the same process.
While it may work in some configurations, it has not been thoroughly tested, and you may encounter issues with IPC via `Iceoryx2`.
Furthermore, no guarantees are made about using `Go2Controller` instances across **multiple processes** (for the same reasons).

For now, it is recommended to have only one instance of the `Go2Controller` alive across your whole system.
Support for multiple instances across processes is being considered, but it is not currently a priority.
If there is sufficient demand or a stronger use case, it can become a higher-priority feature in the future.


## `Illegal instruction (core dumped)` when running example scripts

This issue occurs with an external package. It's likely related to precompiled wheels that use vector extensions (e.g. AVX/AVX2/AVX-512/SEE4.2/ARM-specific) built for specific hardware, which may not match the CPU (under or not under a virtualized environment) this is running on.

I can't determine which package is causing this, since it could really be any package. However, I have encountered this issue before with two packages: `opencv` and `cryptopgraphy`.


### OpenCV

You can either build `opencv` from source (last resort) or install a known working version. Installing `opencv` with the following versions resolved the error on my machine.

```bash
pip install "opencv-python==4.10.0.84" "opencv-contrib-python==4.10.0.84"
```

### Cryptography 

This one is a bit more *sneaky*, since it isn't an explicit import: `aiortc` pulls the `cryptography` package transitively. `cryptography/x509/certificate_transparency.py` pulls `cryptography.hazmat.bindings._rust` (Rust compiled extensions) on line 8, which causes the error. On the bright side, we can compile and build the package locally.

Assuming you have `rust` installed (which you should, if you installed `iceoryx2`), run the following.

```bash
pip uninstall -y cryptography
pip install --no-binary cryptography cryptography
```

### Other packages

You can run the following command to see exactly what is causing the error.

```bash
python3 -X faulthandler my_script.py
```

Likely, it will be caused by a python package importing pre-compiled bindings (from a wheel), written in a different language.