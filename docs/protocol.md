# Telemetry Protocol (CSV)

Firmware streams text lines over UART2 @ 115200.

## Header
t_ms,ax,ay,az,gx,gy,gz,acc_ok


## Data line (example)
12345,0.01,-0.02,0.98,1.20,-0.40,0.10,1

- `t_ms`: HAL_GetTick() in ms
- `ax,ay,az`: acceleration in g
- `gx,gy,gz`: angular rate in deg/s
- `acc_ok`: 1 if accel magnitude roughly looks like ~1g (quick sanity flag)

## Error line
If a read fails, firmware may output:
err,t_ms,code
Example:
err,12400,1