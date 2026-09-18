#pragma once
// TODO(Part 6.1): CameraPipeline — capture -> undistort -> push to frameBus. Opens the UVC device
// through a GStreamer pipeline string (Part 6.2) rather than the plain V4L2 backend, for reliable
// format negotiation over usbipd-win. Does no ML or rendering work in this thread.
//
// Filled in during Phase 4.
