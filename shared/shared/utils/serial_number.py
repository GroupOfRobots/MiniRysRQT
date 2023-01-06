def setWidgetSerialNumber(context, widget):
    if context.serial_number() > 1:
        widget.setWindowTitle(
            widget.windowTitle() + (' (%d)' % context.serial_number()))