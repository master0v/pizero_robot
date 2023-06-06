# pizero_robot

This is work-in-progress.  Please star and come back when I am done :)

If you are interested in contributing, please email alexei@masterov.us

# installing the telegram_control.service

```
sudo cp telegram_control.service /etc/systemd/system/

sudo systemctl daemon-reload && sudo systemctl enable telegram_control.service

sudo systemctl start telegram_control.service && sudo journalctl -f -u telegram_control.service
```
