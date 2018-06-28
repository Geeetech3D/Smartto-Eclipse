#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import time
import math
import logging
import threading
import ttk
import tkFileDialog as filedialog
from Tkinter import *
from ScrolledText import *
from Helper.DeviceHelper import DeviceHelper
from Helper.SerialHelper import SerialHelper
from Protocol.YModem import YModem

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(levelname)s %(message)s',
                    datefmt='%H:%M:%S')

timeout_count = 0
timer = None

class FirmwareUpdator(object):
    def __init__(self, output):
        self.serial_list = None
        self.ser = None
        self.output_panel = output

    def update_firmware(self, profile, com, mainboard):
        self.output_to_panel("**********Firmware Upgrade Process**********\n")

        try:
            self.output_to_panel("Initial Global Configuration...\n")
            self.init_config(profile, com)
            
            self.output_to_panel("Enter Boot Mode...\n")
            self.set_boot()
            self.output_to_panel("Start Upgrade Mainboard...\n")
            self.upload_mainboard(mainboard)
            
            self.output_to_panel("Restart System...\n")
            self.reconnect_serial()
            self.output_to_panel("Success!\n")
            self.close_serial()

        except Exception as e:
            self.output_to_panel("Error, upgrading failed! Detail: " + str(e) + "\n")
            return

        
    def output_to_panel(self, message):
        self.output_panel.insert(END, message)
        self.output_panel.see(END)

    def init_config(self, profile, com):
        if com == "Empty":
            raise Exception("No port was detected!")
        self.port = com
        logging.debug(self.port + ' was chosen')

        config = None
        try:
            with open(profile, 'r') as conf_file:
                config = dict(line.strip().split(':') for line in conf_file if line and line.split())
            
            self.baudrate = config['baudrate']
            self.parity = config['parity']
            self.databit = config['databit']
            self.stopbit = config['stopbit']
            
        except Exception as e:
            #logging.error(e)
            #raise Exception("Configuration file broken!")
            raise e

    def set_boot(self):
        global timeout_count
        global timer

        def check_receive_timeout():
            global timer
            global timeout_count
            timeout_count += 1
            timer = threading.Timer(1, check_receive_timeout)
            timer.start()
        timer = threading.Timer(1, check_receive_timeout)
        timer.start()

        while True:
            if timeout_count > 6:
                timer.cancel()
                timeout_count = 0
                raise Exception("Opening com timeout，you may chose a wrong port!")

            try:
                self.open_serial()
            except Exception as e:
                timer.cancel()
                timeout_count = 0
                raise Exception(e)

            # clear the tunnel
            self.start_on_data_received(self.data_received_handler)
            time.sleep(0.3)
            self.ser.write('c') 
            time.sleep(0.1)
            self.ser.write('1')
            time.sleep(0.1)

            self.stop_serial_listener()

            timer.cancel()
            timeout_count = 0
            break

    def reconnect_serial(self):
        self.ser.reconnect()

    '''
    打开串口，并添加相应的设置
    '''
    def open_serial(self):
        if self.ser is not None:
            self.ser.stop_on_data_received()
            self.ser.disconnect()
        self.ser = SerialHelper(Port=self.port,
                                BaudRate=self.baudrate,
                                ByteSize=self.databit,
                                Parity=self.parity,
                                Stopbits=self.stopbit)
        self.ser.connect()
        self.ser.start_on_connected_changed()


    '''
    串口接收数据的处理器
    '''
    def data_received_handler(self, data):
        logging.debug(str(data))

    '''
    关闭串口
    '''
    def close_serial(self):
        self.ser.disconnect()

    '''
    开启初始的串口监控
    '''
    def start_on_data_received(self, func):
        self.ser.start_on_data_received(func)

    '''
    关闭初始的串口监控（即关闭boot模式期间的监视器）
    '''
    def stop_serial_listener(self):
        self.ser.stop_on_data_received()


    def upload_mainboard(self, mainboard):
        parent = self
        def getc(size):
            return parent.ser._serial.read(size) or None
        def putc(data):
            return parent.ser._serial.write(data)
        modem = YModem(getc, putc)
        try:
            stream = open(mainboard, 'rb')
            length = os.path.getsize(mainboard)
        except IOError as e:
            # logging.error(e)
            raise Exception("Failed to open firmware configuration file!")
        try:
            modem.send(stream, length, self.data_received_handler, 8, self.record_progress)
        except Exception as e:
            stream.close()
            raise
        stream.close()

    def record_progress(self, total_count, ok_count):
        self.output_to_panel("Upgrading... " + str(math.trunc(ok_count * 100 / total_count)) + '% \n')


class Application(Frame):
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.serial_list = None
        self.createWindow(master, 680, 350)
        self.createWidget(master)

    def createWindow(self, root, width, height):
        screenwidth = root.winfo_screenwidth()  
        screenheight = root.winfo_screenheight()  
        size = '%dx%d+%d+%d' % (width, height, (screenwidth - width)/2, (screenheight - height)/2)
        root.geometry(size)
        root.maxsize(width, height)
        root.minsize(width, height)

    def createWidget(self, master):

        self.device_helper = DeviceHelper()
        self.device_helper.find_all_devices(self)
        self.menu = Frame(master, height=10, background="#f2f2f2")
        self.menu.pack(side=TOP, fill=X, pady=5)

        self.display_board = Frame(master, background="#f2f2f2")
        self.display_board.pack(side=TOP, fill=X)

        self.lb_port_list = Label(self.menu, text="Port:", width=4, background="#f2f2f2")
        self.lb_port_list.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.port_item = StringVar()
        self.cb_port_list = ttk.Combobox(self.menu, width=20, textvariable=self.port_item, state="readonly", background="#f2f2f2")
        self.cb_port_list["values"] = self.serial_list or ("Empty")
        self.cb_port_list.current(0)
        self.cb_port_list.pack(side=LEFT, anchor="w", pady=5)

        self.btn_select = Button(self.menu, text="Select", height=1, width=13, command=self.select_firmware, bg="#f2f2f2")
        self.btn_select.pack(side=LEFT, anchor="w", padx=10)


        self.btn_upgrade = Button(self.menu, text="Upgrade", height=1, width=13, command=self.create_upgrade_thread, bg="#f2f2f2")
        self.btn_upgrade.pack(side=LEFT, anchor="w", padx=10)

        self.btn_clear = Button(self.menu, text="Clear", height=1, width=8, command=self.clear_info_text, bg="#f2f2f2")
        self.btn_clear.pack(side=LEFT, anchor="w", padx=10)

        self.info_text = ScrolledText(self.display_board, state="normal", width=93)
        self.info_text.bind("<KeyPress>", lambda e: "break")
        self.info_text.pack(side=LEFT, anchor="n", padx=5, fill=Y)

    def select_firmware(self):
        self.file_path = filedialog.askopenfilename(filetypes = [('BIN', 'bin')])

    def clear_info_text(self):
        self.info_text.delete(1.0, END)
    
    def create_upgrade_thread(self):
        tConnected = threading.Thread(target=self.start_to_upgrade, args=())
        tConnected.setDaemon(True)
        tConnected.start()

    def start_to_upgrade(self):
        self.btn_upgrade["state"] = "disabled"
        updator = FirmwareUpdator(self.info_text)
        profile = os.path.join(os.path.split(os.path.realpath(__file__))[0], "default.conf")
        updator.update_firmware(profile, self.port_item.get(), self.file_path)
        self.btn_upgrade["state"] = "normal"


if __name__ == '__main__':
    master = Tk()
    master.configure(background="#f2f2f2")
    master.title('Smatto Firmware Tool')
    master.iconbitmap(os.path.join(os.path.split(os.path.realpath(__file__))[0], "tool.ico"))
    app = Application(master)
    app.mainloop()






