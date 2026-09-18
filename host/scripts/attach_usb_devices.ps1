# TODO(Part 2.2): Windows-side usbipd bind/attach helper.
#
# WSL2 loses USB device attachments on every restart, so this is run at the start of every dev
# session from an ADMIN PowerShell prompt on the Windows side (not from inside WSL2). It binds and
# attaches all three devices in one command: the USB camera, the USB-to-Ethernet adapter for the
# LiDAR, and the STM32 hub's USB-serial port.
#
# Filled in during Phase 0, once the three bus IDs are known from `usbipd list`.
Write-Error "TODO: see docs/BUILD_GUIDE.md Part 2.2"
