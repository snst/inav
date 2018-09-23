"C:\work\OpenOCD\bin\openocd.exe" -f interface/stlink.cfg -f target/stm32f1x.cfg -c init -c "reset init" -c "echo **READY**"
