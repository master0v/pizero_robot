#!/home/aim/myenv/bin/python

# 1. install aiogram 3.0
# pip install -U --pre aiogram
# pip3 install rpi_ws281x

# 2. run with sudo

import os, sys
import ledControl
lc=ledControl.ledControl()
lc.blueColorWipe()
lc.wipeClean()

import logging

from aiogram import Bot, Dispatcher, executor, types

# Configure logging
logging.basicConfig(level=logging.INFO)

BOT_TOKEN = os.getenv("TELEGRAM_BOT_TOKEN")
if (BOT_TOKEN is None):
  print("run 'sudo -E ./telegram_control.py' to preserve TELEGRAM_BOT_TOKEN environmental variable")
  sys.exit(1)

# Initialize bot and dispatcher
bot = Bot(token=BOT_TOKEN)
dp = Dispatcher(bot)


@dp.message_handler(commands=['start', 'help'])
async def send_welcome(message: types.Message):
    """
    This handler will be called when user sends `/start` or `/help` command
    """
    await message.reply("Hi!\nI'm EchoBot!\nPowered by aiogram.")



@dp.message_handler()
async def echo(message: types.Message):
    # old style:
    # await bot.send_message(message.chat.id, message.text)
    if message.text.lower() == "red":
      lc.redColorWipe()
    elif message.text.lower() == "green":
      lc.greenColorWipe()
    elif message.text.lower() == "blue":
      lc.blueColorWipe()
    elif message.text.lower() == "off":
      lc.wipeClean()
    else:
      await message.answer("'" + message.text +
        "' is not understood.  I understand 'red', 'green', 'blue' or 'off'")


if __name__ == '__main__':
    executor.start_polling(dp, skip_updates=True)