# power button state notification message

uint64 timestamp			    # time since system start (microseconds)

uint8 PWR_BUTTON_STATE_IDEL = 0             # Button went up without meeting shutdown button down time (delete event)    按钮上升但没有会议关闭按钮关闭时间（删除事件）
uint8 PWR_BUTTON_STATE_DOWN = 1             # Button went Down  按钮坏了
uint8 PWR_BUTTON_STATE_UP = 2               # Button went Up    按钮上升
uint8 PWR_BUTTON_STATE_REQUEST_SHUTDOWN = 3 # Button went Up after meeting shutdown button down time  按钮在会议结束后上升按钮关闭时间

uint8 event                                 # one of PWR_BUTTON_STATE_*
