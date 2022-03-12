Compile with `colcon build`.

[To tune the Cyclone DDS to support large messages (10 MB)](https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html#cyclone-dds-tuning),
increase the maximum buffer size with
`sudo sysctl -w net.core.rmem_max=2147483647`.
