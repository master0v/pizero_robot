#!/usr/bin/python3
#  -*- coding: utf-8 -*-

# pip3 install telegram

from time import sleep
import signal
import re
from subprocess import check_output
import telegram

import logging
logging.basicConfig(format='%(name)-8s: %(levelname)-6s %(message)s') # %(asctime)-10s
logger = logging.getLogger('telegram.bot')
logger.setLevel(logging.INFO)
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)


import os
BOT_TOKEN = os.getenv("TELEGRAM_BOT_TOKEN")
CHAT_ID = os.getenv("TELEGRAM_CHAT_ID")


from ledControl import ledControl

logger.debug("Turning on LED")
lc=ledControl()
lc.redColorWipe()

# get ip address
attempt = 0
while True:
  logger.debug(f"Getting the IP using 'ifconfig' (attempt {attempt})")
  wlan0 = str(check_output(["ifconfig", "wlan0"]))
  matches = re.findall(r'inet \d+[.]\d+[.]\d+[.]\d+', wlan0) 
  if matches:
    ip = matches[0].split()[1]
    logger.debug(f" - got {ip}")
    break
  elif attempt > 10:
    ip = "Can't get ip"
    break
  attempt+=1
  
lc.blueColorWipe()

# send it to telegram
logger.debug(f"sending '{ip}' to Telegram")
bot = telegram.Bot(token=BOT_TOKEN)
try:
  bot.send_message(chat_id=CHAT_ID, text=f"RaspiTank got IP {ip}")
except Exception as e:
  logger.warning(f"Can't send to telegram: {e}")

lc.redColorWipe()

# Termination signals callback
def exit_gracefully(signum, frame):
  #print(f"signum: {signum}, frame: {frame}")
  logger.info(f"Caught termination signal {signum}. Exiting cleanly :)")
  lc.wipeClean()
  exit(0)

signal.signal(signal.SIGINT, exit_gracefully) # stop-sigterm
signal.signal(signal.SIGTERM, exit_gracefully)

while True:
  logger.debug(".")
  sleep(10)
