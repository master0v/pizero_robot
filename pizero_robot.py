#!/usr/bin/python3
#  -*- coding: utf-8 -*-

from time import sleep
import signal
import re
from subprocess import check_output
import telegram

import telegram_token
from ledControl import ledControl



lc=ledControl()
lc.redColorWipe()

sleep(3)

# get ip address
wlan0 = str(check_output(["ifconfig", "wlan0"]))
matches = re.findall(r'inet \d+[.]\d+[.]\d+[.]\d+', wlan0) 
if matches:
  ip = matches[0].split()[1]
else:
  ip = "Can't get ip"

# send it to telegram
bot = telegram.Bot(token=telegram_token.BOT_TOKEN)
bot.send_message(chat_id=telegram_token.CHAT_ID, text=f"RaspiTank got IP {ip}")

# Termination signals callback
def exit_gracefully(signum, frame):
  #print(f"signum: {signum}, frame: {frame}")
  print(f"Caught termination signal {signum}. Exiting cleanly :)")
  lc.wipeClean()
  exit(0)

signal.signal(signal.SIGINT, exit_gracefully) # stop-sigterm
signal.signal(signal.SIGTERM, exit_gracefully)

while True:
  print(".")
  sleep(3)
