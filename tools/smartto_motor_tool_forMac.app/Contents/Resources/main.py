# !/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import time
import logging
import threading
import ttk
from Tkinter import *
from ScrolledText import *
from Helper.DeviceHelper import DeviceHelper
from Helper.SerialHelper import SerialHelper

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(levelname)s %(message)s',
                    datefmt='%H:%M:%S')

class MotorController(object):
    def __init__(self, output):
        self.serial_list = None
        self.output_panel = output

    def init_config(self, profile, com, show_ports):
        if com == "Empty":
            raise Exception("No port was detected")
        self.port = com
        logging.debug(self.port + ' was chosen')

        config = None
        try:
            with open(profile, 'r') as conf_file:
                config = dict(line.strip().split(':') for line in conf_file if line)
            
            # 获取串口参数并保存到类成员变量中
            self.baudrate = config['baudrate']
            self.parity = config['parity']
            self.databit = config['databit']
            self.stopbit = config['stopbit']
            
        except Exception as e:
            # logging.error(e)       
            raise Exception("There were serveral wrong data in profile!")

    def open_serial(self):
        self.ser = SerialHelper(Port=self.port,
                                BaudRate=self.baudrate,
                                ByteSize=self.databit,
                                Parity=self.parity,
                                Stopbits=self.stopbit)
        self.ser.connect()
        self.ser.on_connected_changed()

    def close_serial(self):
        self.ser.disconnect()

    def send_cmd(self, msg, parent):
        if hasattr(self, "ser"):
            self.ser.write(msg + '\n')
            self.output_to_panel("Custom command: \"" + msg + "\" has been sent!\n")
        else:
            self.output_to_panel("Error, Setting motor failed! Details: Please choose a port and connect to your machine!\n")

    def send_motor_cmd(self, mode, parent):
        if hasattr(self, "ser"):
            if mode == 1:
                self.ser.write("M2003 X" + parent.travel_x.get() + " Y" + parent.travel_y.get() + " Z" + parent.travel_z.get() + "\n")
                self.wait_for_ok()
                self.output_to_panel("Travel set! Details: " + parent.travel_x.get() + " " + parent.travel_y.get() + " " + parent.travel_z.get() + "\n")
            elif mode == 2:
                self.ser.write("M2004 X" + parent.step_x.get() + " Y" + parent.step_y.get() + " Z" + parent.step_z.get() + " E" + parent.step_e.get() + "\n")
                self.wait_for_ok()
                self.output_to_panel("Step set! Details: " + parent.step_x.get() + " " + parent.step_y.get() + " " + parent.step_z.get() +  " " + parent.step_e.get() + "\n")
            elif mode == 3:
                self.ser.write("M2005 X " + parent.direction_x.get() + " Y " + parent.direction_y.get() + " Z " + parent.direction_z.get() + " E0 " + parent.direction_e0.get() + " E1 " + parent.direction_e1.get() + " E2 " + parent.direction_e2.get() + "\n")
                self.wait_for_ok()
                self.output_to_panel("Direction set! Details: " + parent.direction_x.get() + " " + parent.direction_y.get() + " " + parent.direction_z.get() +  " " + parent.direction_e0.get() + " " + parent.direction_e1.get() + " " + parent.direction_e2.get() + "\n")
            else:
                self.ser.write("M2006 X" + parent.velocity_x + " Y" + parent.velocity_y + " Z" + parent.velocity_z + " E" + parent.velocity_e + "\n")
                self.wait_for_ok()
                self.output_to_panel("Velocity set! Details: " + parent.velocity_x.get() + " " + parent.velocity_y.get() + " " + parent.velocity_z.get() +  " " + parent.velocity_e.get() + "\n")
        else:
                raise Exception("Please choose a port and connect to your machine!")

    def get_motor_status(self, parent):
        if hasattr(self, "ser"):
            self.ser.write("M2002\n")
            self.wait_for_status()
            parent.travel_x.set(self.s_config["max_position[X_AXIS]"])
            parent.travel_y.set(self.s_config["max_position[Y_AXIS]"])
            parent.travel_z.set(self.s_config["max_position[Z_AXIS]"])
            parent.step_x.set(self.s_config["steps_per_mm[X_AXIS]"])
            parent.step_y.set(self.s_config["steps_per_mm[Y_AXIS]"])
            parent.step_z.set(self.s_config["steps_per_mm[Z_AXIS]"])
            parent.step_e.set(self.s_config["steps_per_mm[E_AXIS]"])
            parent.direction_x.set(self.s_config["motor_direction[X_AXIS]"])
            parent.direction_y.set(self.s_config["motor_direction[Y_AXIS]"])
            parent.direction_z.set(self.s_config["motor_direction[Z_AXIS]"])
            parent.direction_e0.set(self.s_config["motor_direction[E_AXIS]"])
            parent.direction_e1.set(self.s_config["motor_direction[E1_AXIS]"])
            parent.direction_e2.set(self.s_config["motor_direction[E2_AXIS]"])
            parent.velocity_x.set(self.s_config["max_feedrate[X_AXIS]"])
            parent.velocity_y.set(self.s_config["max_feedrate[Y_AXIS]"])
            parent.velocity_z.set(self.s_config["max_feedrate[Z_AXIS]"])
            parent.velocity_e.set(self.s_config["max_feedrate[E_AXIS]"])
            self.output_to_panel("Motor status received!")
        else:
            self.output_to_panel("Error, Getting motor failed! Details: Please choose a port and connect to your machine!\n")
    
    def wait_for_ok(self):
        def check_receive_timeout():
            global timer
            global timeout_count
            timeout_count += 1
            timer = threading.Timer(1, check_receive_timeout)
            timer.start()
        timeout_count = 0
        timer = threading.Timer(1, check_receive_timeout)
        while True:
            if timeout_count > 5:
                timer.cancel()
                raise Exception("Receiving data response timeout!")
            line = self.ser._serial.readline()
            if line == None:
                continue
            elif 'ok' in line:
                timer.cancel()
                break
            else:
                continue
    
    def wait_for_status(self):
        def check_receive_timeout():
            global timer
            global timeout_count
            timeout_count += 1
            timer = threading.Timer(1, check_receive_timeout)
            timer.start()
        timeout_count = 0
        timer = threading.Timer(1, check_receive_timeout)
        while True:
            if timeout_count > 8:
                timer.cancel()
                raise Exception("Receiving data response timeout!")
            line = self.ser._serial.readline()
            if line == None:
                continue
            elif 'AXIS' in line:
                timer.cancel()
                config = line.split(";")
                config.pop()
                self.s_config = dict(item.strip().split(":") for item in config if item)
                break
            else:
                continue
        

    def set_normal(self):
        try:
            self.open_serial()
        except Exception as e:
            raise Exception("Cannot make serial port object!")
        self.start_on_data_received(self.data_received_handler)
        time.sleep(3)
        self.stop_serial_listener()

    def set_motor(self, mode, parent):
        try:
            self.output_to_panel("Start to send motor command...\n")
            self.send_motor_cmd(mode, parent)
        except Exception as e:
            self.output_to_panel("Error, Setting motor failed! Details: " + str(e) + "\n")
            return

    def output_to_panel(self, message):
        self.output_panel.insert(END, message)
        self.output_panel.see(END)

    def start_on_data_received(self, func):
        self.ser.start_on_data_received(func)

    def stop_serial_listener(self):
        self.ser.stop_on_data_received()

    def data_received_handler(self, data):
        logging.debug(str(data))

    def reconnect_serial(self):
        self.ser.reconnect()

class Application(Frame):
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.serial_list = None
        self.createWindow(master, 1000, 680)
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

        self.menu = Frame(master, background="#f2f2f2")
        self.menu.pack(side=TOP, fill=X)

        self.lb_port_list = Label(self.menu, text="Serial Port:", width=10, bg="#f2f2f2")
        self.lb_port_list.pack(side=LEFT, anchor="n", padx=5, pady=5)

        self.port_item = StringVar()
        self.cb_port_list = ttk.Combobox(self.menu, width=25, textvariable=self.port_item, state="readonly")
        self.cb_port_list["values"] = self.serial_list or ("Empty")
        self.cb_port_list.current(0)
        self.cb_port_list.pack(side=LEFT, anchor="n", pady=5)

        self.btn_get = Button(self.menu, text=" Get Status",  bitmap="gray12", compound=LEFT, anchor="w",   height=15, width=95, command=self.get_motor_status)
        self.btn_get.pack(side=RIGHT, anchor="n", padx=5, pady=5)

        self.btn_reconnect = Button(self.menu, text=" Reconnect",  bitmap="gray12", compound=LEFT, anchor="w",   height=15, width=95, command=self.reconnect)
        self.btn_reconnect.pack(side=RIGHT, anchor="n", padx=5, pady=5)

        self.btn_disconnect = Button(self.menu, text=" Disconnect", bitmap="gray12", compound=LEFT, anchor="w",   height=15, width=95, command=self.disconnect)
        self.btn_disconnect.pack(side=RIGHT, anchor="n", padx=5, pady=5)

        self.btn_connect = Button(self.menu, text=" Connect",  bitmap="gray12", compound=LEFT, anchor="w",   height=15, width=95, command=self.connect)
        self.btn_connect.pack(side=RIGHT, anchor="n", padx=5, pady=5)

        self.upper_panel = Frame(master, background="#f2f2f2")
        self.upper_panel.pack(side=TOP, fill=X)

        self.middle_panel = Frame(master, background="#f2f2f2")
        self.middle_panel.pack(side=TOP, fill=X)

        self.lower_panel = Frame(master, background="#f2f2f2")
        self.lower_panel.pack(side=TOP, fill=X)

        self.travel_board = Frame(self.upper_panel, background="#f2f2f2")
        self.travel_board.pack(side=TOP, fill=X)
        
        self.lb_travel = Label(self.travel_board, text="Motor Travel", anchor = 'w', width=12, bg="#f2f2f2")
        self.lb_travel.pack(side=LEFT, anchor="n", padx=10, pady=5)

        self.lb_travel_x = Label(self.travel_board, text="X", width=2, bg="#f2f2f2")
        self.lb_travel_x.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.travel_x = StringVar()
        self.et_travel_x = Entry(self.travel_board, textvariable=self.travel_x, width=8, state="normal")
        self.et_travel_x.pack(side=LEFT, anchor="n", padx=2, pady=5)
        
        self.lb_travel_y = Label(self.travel_board, text="Y", width=2, bg="#f2f2f2")
        self.lb_travel_y.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.travel_y = StringVar()
        self.et_travel_y = Entry(self.travel_board, textvariable=self.travel_y, width=8, state="normal")
        self.et_travel_y.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_travel_z = Label(self.travel_board, text="Z", width=2, bg="#f2f2f2")
        self.lb_travel_z.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.travel_z = StringVar()
        self.et_travel_z = Entry(self.travel_board, textvariable=self.travel_z, width=8, state="normal")
        self.et_travel_z.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.btn_travel = Button(self.travel_board, text=" Set Travel ",   bitmap="gray12", compound=LEFT, anchor="w",   height=15, width=95, command=self.set_travel)
        self.btn_travel.pack(side=RIGHT, anchor="n", padx=5, pady=5)

        self.step_board = Frame(self.upper_panel, background="#f2f2f2")
        self.step_board.pack(side=TOP, fill=X)

        self.lb_step = Label(self.step_board, text="Motor Step",  anchor = 'w', width=12, bg="#f2f2f2")
        self.lb_step.pack(side=LEFT, anchor="n", padx=10, pady=5)

        self.lb_step_x = Label(self.step_board, text="X", width=2, bg="#f2f2f2")
        self.lb_step_x.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.step_x = StringVar()
        self.et_step_x = Entry(self.step_board, textvariable=self.step_x, width=8, state="normal")
        self.et_step_x.pack(side=LEFT, anchor="n", padx=2, pady=5)
        
        self.lb_step_y = Label(self.step_board, text="Y", width=2, bg="#f2f2f2")
        self.lb_step_y.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.step_y = StringVar()
        self.et_step_y = Entry(self.step_board, textvariable=self.step_y, width=8, state="normal")
        self.et_step_y.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_step_z = Label(self.step_board, text="Z", width=2, bg="#f2f2f2")
        self.lb_step_z.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.step_z = StringVar()
        self.et_step_z = Entry(self.step_board, textvariable=self.step_z, width=8, state="normal")
        self.et_step_z.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_step_e = Label(self.step_board, text="E", width=2, bg="#f2f2f2")
        self.lb_step_e.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.step_e = StringVar()
        self.et_step_e = Entry(self.step_board, textvariable=self.step_e, width=8, state="normal")
        self.et_step_e.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.btn_step = Button(self.step_board, text=" Set Step ",  bitmap="gray12", compound=LEFT, anchor="w",   height=15, width=95, command=self.set_step)
        self.btn_step.pack(side=RIGHT, anchor="n", padx=5, pady=5)

        self.direction_board = Frame(self.upper_panel, background="#f2f2f2")
        self.direction_board.pack(side=TOP, fill=X)

        self.lb_direction = Label(self.direction_board, text="Motor Direction", anchor = 'w', width=12, bg="#f2f2f2")
        self.lb_direction.pack(side=LEFT, anchor="n", padx=10, pady=5)

        self.lb_direction_x = Label(self.direction_board, text="X", width=2, bg="#f2f2f2")
        self.lb_direction_x.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.direction_x = StringVar()
        self.et_direction_x = Entry(self.direction_board, textvariable=self.direction_x, width=8, state="normal")
        self.et_direction_x.pack(side=LEFT, anchor="n", padx=2, pady=5)
        
        self.lb_direction_y = Label(self.direction_board, text="Y", width=2, bg="#f2f2f2")
        self.lb_direction_y.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.direction_y = StringVar()
        self.et_direction_y = Entry(self.direction_board, textvariable=self.direction_y, width=8, state="normal")
        self.et_direction_y.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_direction_z = Label(self.direction_board, text="Z", width=2, bg="#f2f2f2")
        self.lb_direction_z.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.direction_z = StringVar()
        self.et_direction_z = Entry(self.direction_board, textvariable=self.direction_z, width=8, state="normal")
        self.et_direction_z.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_direction_e0 = Label(self.direction_board, text="E0", width=2, bg="#f2f2f2")
        self.lb_direction_e0.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.direction_e0 = StringVar()
        self.et_direction_e0 = Entry(self.direction_board, textvariable=self.direction_e0, width=8, state="normal")
        self.et_direction_e0.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_direction_e1 = Label(self.direction_board, text="E1", width=2, bg="#f2f2f2")
        self.lb_direction_e1.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.direction_e1 = StringVar()
        self.et_direction_e1 = Entry(self.direction_board, textvariable=self.direction_e1, width=8, state="normal")
        self.et_direction_e1.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_direction_e2 = Label(self.direction_board, text="E2", width=2, bg="#f2f2f2")
        self.lb_direction_e2.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.direction_e2 = StringVar()
        self.et_direction_e2 = Entry(self.direction_board, textvariable=self.direction_e2, width=8, state="normal")
        self.et_direction_e2.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.btn_direction = Button(self.direction_board, text=" Set Direction ",   bitmap="gray12", compound=LEFT, anchor="w",   height=15, width=95, command=self.set_direction)
        self.btn_direction.pack(side=RIGHT, anchor="n", padx=5, pady=5)

        self.velocity_board = Frame(self.upper_panel, background="#f2f2f2")
        self.velocity_board.pack(side=TOP, fill=X)

        self.lb_velocity = Label(self.velocity_board, text="Motor Velocity", anchor = 'w', width=12, bg="#f2f2f2")
        self.lb_velocity.pack(side=LEFT, anchor="n", padx=10, pady=5)

        self.lb_velocity_x = Label(self.velocity_board, text="X", width=2, bg="#f2f2f2")
        self.lb_velocity_x.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.velocity_x = StringVar()
        self.et_velocity_x = Entry(self.velocity_board, textvariable=self.velocity_x, width=8, state="normal")
        self.et_velocity_x.pack(side=LEFT, anchor="n", padx=2, pady=5)
        
        self.lb_velocity_y = Label(self.velocity_board, text="Y", width=2, bg="#f2f2f2")
        self.lb_velocity_y.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.velocity_y = StringVar()
        self.et_velocity_y = Entry(self.velocity_board, textvariable=self.velocity_y, width=8, state="normal")
        self.et_velocity_y.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_velocity_z = Label(self.velocity_board, text="Z", width=2, bg="#f2f2f2")
        self.lb_velocity_z.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.velocity_z = StringVar()
        self.et_velocity_z = Entry(self.velocity_board, textvariable=self.velocity_z, width=8, state="normal")
        self.et_velocity_z.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_velocity_e = Label(self.velocity_board, text="E", width=2, bg="#f2f2f2")
        self.lb_velocity_e.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.velocity_e = StringVar()
        self.et_velocity_e = Entry(self.velocity_board, textvariable=self.velocity_e, width=8, state="normal")
        self.et_velocity_e.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.btn_velocity = Button(self.velocity_board, text=" Set Velocity ",  bitmap="gray12", compound=LEFT, anchor="w",   height=15, width=95, command=self.set_velocity)
        self.btn_velocity.pack(side=RIGHT, anchor="n", padx=5, pady=5)

        ##########Max Printing Acceleration##########

        self.max_printing_acceleration_board = Frame(self.upper_panel, background="#f2f2f2")
        self.max_printing_acceleration_board.pack(side=TOP, fill=X)

        self.lb_max_printing_acceleration = Label(self.max_printing_acceleration_board, text="Max printing move acceleration", anchor = 'w', width=28, bg="#f2f2f2")
        self.lb_max_printing_acceleration.pack(side=LEFT, anchor="n", padx=10, pady=5)

        self.lb_max_printing_acceleration_x = Label(self.max_printing_acceleration_board, text="X", width=2, bg="#f2f2f2")
        self.lb_max_printing_acceleration_x.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.max_printing_acceleration_x = StringVar()
        self.et_max_printing_acceleration_x = Entry(self.max_printing_acceleration_board, textvariable=self.max_printing_acceleration_x, width=8, state="normal")
        self.et_max_printing_acceleration_x.pack(side=LEFT, anchor="n", padx=2, pady=5)
        
        self.lb_max_printing_acceleration_y = Label(self.max_printing_acceleration_board, text="Y", width=2, bg="#f2f2f2")
        self.lb_max_printing_acceleration_y.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.max_printing_acceleration_y = StringVar()
        self.et_max_printing_acceleration_y = Entry(self.max_printing_acceleration_board, textvariable=self.max_printing_acceleration_y, width=8, state="normal")
        self.et_max_printing_acceleration_y.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_max_printing_acceleration_z = Label(self.max_printing_acceleration_board, text="Z", width=2, bg="#f2f2f2")
        self.lb_max_printing_acceleration_z.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.max_printing_acceleration_z = StringVar()
        self.et_max_printing_acceleration_z = Entry(self.max_printing_acceleration_board, textvariable=self.max_printing_acceleration_z, width=8, state="normal")
        self.et_max_printing_acceleration_z.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_max_printing_acceleration_e = Label(self.max_printing_acceleration_board, text="E", width=2, bg="#f2f2f2")
        self.lb_max_printing_acceleration_e.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.max_printing_acceleration_e = StringVar()
        self.et_max_printing_acceleration_e = Entry(self.max_printing_acceleration_board, textvariable=self.max_printing_acceleration_e, width=8, state="normal")
        self.et_max_printing_acceleration_e.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.btn_max_printing_acceleration = Button(self.max_printing_acceleration_board, text=" Set Acceleration(P) ",  bitmap="gray12", compound=LEFT, anchor="w",   height=15, width=130, command=self.set_max_printing_acceleration)
        self.btn_max_printing_acceleration.pack(side=RIGHT, anchor="n", padx=5, pady=5)

        ##########Max Traveling Acceleration##########

        self.max_traveling_acceleration_board = Frame(self.upper_panel, background="#f2f2f2")
        self.max_traveling_acceleration_board.pack(side=TOP, fill=X)

        self.lb_max_traveling_acceleration = Label(self.max_traveling_acceleration_board, text="Max traveling move acceleration", anchor = 'w', width=28, bg="#f2f2f2")
        self.lb_max_traveling_acceleration.pack(side=LEFT, anchor="n", padx=10, pady=5)

        self.lb_max_traveling_acceleration_x = Label(self.max_traveling_acceleration_board, text="X", width=2, bg="#f2f2f2")
        self.lb_max_traveling_acceleration_x.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.max_traveling_acceleration_x = StringVar()
        self.et_max_traveling_acceleration_x = Entry(self.max_traveling_acceleration_board, textvariable=self.max_traveling_acceleration_x, width=8, state="normal")
        self.et_max_traveling_acceleration_x.pack(side=LEFT, anchor="n", padx=2, pady=5)
        
        self.lb_max_traveling_acceleration_y = Label(self.max_traveling_acceleration_board, text="Y", width=2, bg="#f2f2f2")
        self.lb_max_traveling_acceleration_y.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.max_traveling_acceleration_y = StringVar()
        self.et_max_traveling_acceleration_y = Entry(self.max_traveling_acceleration_board, textvariable=self.max_traveling_acceleration_y, width=8, state="normal")
        self.et_max_traveling_acceleration_y.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_max_traveling_acceleration_z = Label(self.max_traveling_acceleration_board, text="Z", width=2, bg="#f2f2f2")
        self.lb_max_traveling_acceleration_z.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.max_traveling_acceleration_z = StringVar()
        self.et_max_traveling_acceleration_z = Entry(self.max_traveling_acceleration_board, textvariable=self.max_traveling_acceleration_z, width=8, state="normal")
        self.et_max_traveling_acceleration_z.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_max_traveling_acceleration_e = Label(self.max_traveling_acceleration_board, text="E", width=2, bg="#f2f2f2")
        self.lb_max_traveling_acceleration_e.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.max_traveling_acceleration_e = StringVar()
        self.et_max_traveling_acceleration_e = Entry(self.max_traveling_acceleration_board, textvariable=self.max_traveling_acceleration_e, width=8, state="normal")
        self.et_max_traveling_acceleration_e.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.btn_max_traveling_acceleration = Button(self.max_traveling_acceleration_board, text=" Set Acceleration(T) ",  bitmap="gray12", compound=LEFT, anchor="w",   height=15, width=130, command=self.set_max_traveling_acceleration)
        self.btn_max_traveling_acceleration.pack(side=RIGHT, anchor="n", padx=5, pady=5)

        ##########Extruder PID##########

        self.extruder_pid_board = Frame(self.upper_panel, background="#f2f2f2")
        self.extruder_pid_board.pack(side=TOP, fill=X)

        self.lb_extruder_pid = Label(self.extruder_pid_board, text="Extruder PID", anchor = 'w', width=12, bg="#f2f2f2")
        self.lb_extruder_pid.pack(side=LEFT, anchor="n", padx=10, pady=5)

        self.lb_extruder_pid_h = Label(self.extruder_pid_board, text="H", width=2, bg="#f2f2f2")
        self.lb_extruder_pid_h.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.extruder_pid_h = StringVar()
        self.et_extruder_pid_h = Entry(self.extruder_pid_board, textvariable=self.extruder_pid_h, width=8, state="normal")
        self.et_extruder_pid_h.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_extruder_pid_p = Label(self.extruder_pid_board, text="P", width=2, bg="#f2f2f2")
        self.lb_extruder_pid_p.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.extruder_pid_p = StringVar()
        self.et_extruder_pid_p = Entry(self.extruder_pid_board, textvariable=self.extruder_pid_p, width=8, state="normal")
        self.et_extruder_pid_p.pack(side=LEFT, anchor="n", padx=2, pady=5)
        
        self.lb_extruder_pid_i = Label(self.extruder_pid_board, text="I", width=2, bg="#f2f2f2")
        self.lb_extruder_pid_i.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.extruder_pid_i = StringVar()
        self.et_extruder_pid_i = Entry(self.extruder_pid_board, textvariable=self.extruder_pid_i, width=8, state="normal")
        self.et_extruder_pid_i.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_extruder_pid_d = Label(self.extruder_pid_board, text="D", width=2, bg="#f2f2f2")
        self.lb_extruder_pid_d.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.extruder_pid_d = StringVar()
        self.et_extruder_pid_d = Entry(self.extruder_pid_board, textvariable=self.extruder_pid_d, width=8, state="normal")
        self.et_extruder_pid_d.pack(side=LEFT, anchor="n", padx=2, pady=5)


        self.btn_extruder_pid = Button(self.extruder_pid_board, text=" Set Extruder PID ",  bitmap="gray12", compound=LEFT, anchor="w",   height=15, width=130, command=self.set_extruder_pid)
        self.btn_extruder_pid.pack(side=RIGHT, anchor="n", padx=5, pady=5)

        ##########Bed PID##########
        self.bed_pid_board = Frame(self.upper_panel, background="#f2f2f2")
        self.bed_pid_board.pack(side=TOP, fill=X)

        self.lb_bed_pid = Label(self.bed_pid_board, text="Bed PID", anchor = 'w', width=12, bg="#f2f2f2")
        self.lb_bed_pid.pack(side=LEFT, anchor="n", padx=10, pady=5)

        self.lb_bed_pid_p = Label(self.bed_pid_board, text="P", width=2, bg="#f2f2f2")
        self.lb_bed_pid_p.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.bed_pid_p = StringVar()
        self.et_bed_pid_p = Entry(self.bed_pid_board, textvariable=self.bed_pid_p, width=8, state="normal")
        self.et_bed_pid_p.pack(side=LEFT, anchor="n", padx=2, pady=5)
        
        self.lb_bed_pid_i = Label(self.bed_pid_board, text="I", width=2, bg="#f2f2f2")
        self.lb_bed_pid_i.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.bed_pid_i = StringVar()
        self.et_bed_pid_i = Entry(self.bed_pid_board, textvariable=self.bed_pid_i, width=8, state="normal")
        self.et_bed_pid_i.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_bed_pid_d = Label(self.bed_pid_board, text="D", width=2, bg="#f2f2f2")
        self.lb_bed_pid_d.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.bed_pid_d = StringVar()
        self.et_bed_pid_d = Entry(self.bed_pid_board, textvariable=self.bed_pid_d, width=8, state="normal")
        self.et_bed_pid_d.pack(side=LEFT, anchor="n", padx=2, pady=5)


        self.btn_bed_pid = Button(self.bed_pid_board, text=" Set Bed PID ",  bitmap="gray12", compound=LEFT, anchor="w",   height=15, width=95, command=self.set_bed_pid)
        self.btn_bed_pid.pack(side=RIGHT, anchor="n", padx=5, pady=5)

        ##########Max Jerk##########
        self.max_jerk_board = Frame(self.upper_panel, background="#f2f2f2")
        self.max_jerk_board.pack(side=TOP, fill=X)

        self.lb_max_jerk = Label(self.max_jerk_board, text="Max Jerk", anchor = 'w', width=12, bg="#f2f2f2")
        self.lb_max_jerk.pack(side=LEFT, anchor="n", padx=10, pady=5)

        self.lb_max_jerk_x = Label(self.max_jerk_board, text="X", width=2, bg="#f2f2f2")
        self.lb_max_jerk_x.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.max_jerk_x = StringVar()
        self.et_max_jerk_x = Entry(self.max_jerk_board, textvariable=self.max_jerk_x, width=8, state="normal")
        self.et_max_jerk_x.pack(side=LEFT, anchor="n", padx=2, pady=5)
        
        self.lb_max_jerk_y = Label(self.max_jerk_board, text="Y", width=2, bg="#f2f2f2")
        self.lb_max_jerk_y.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.max_jerk_y = StringVar()
        self.et_max_jerk_y = Entry(self.max_jerk_board, textvariable=self.max_jerk_y, width=8, state="normal")
        self.et_max_jerk_y.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_max_jerk_z = Label(self.max_jerk_board, text="Z", width=2, bg="#f2f2f2")
        self.lb_max_jerk_z.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.max_jerk_z = StringVar()
        self.et_max_jerk_z = Entry(self.max_jerk_board, textvariable=self.max_jerk_z, width=8, state="normal")
        self.et_max_jerk_z.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.lb_max_jerk_e = Label(self.max_jerk_board, text="E", width=2, bg="#f2f2f2")
        self.lb_max_jerk_e.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.max_jerk_e = StringVar()
        self.et_max_jerk_e = Entry(self.max_jerk_board, textvariable=self.max_jerk_e, width=8, state="normal")
        self.et_max_jerk_e.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.btn_max_jerk = Button(self.max_jerk_board, text=" Set Max Jerk ",  bitmap="gray12", compound=LEFT, anchor="w",   height=15, width=95, command=self.set_max_jerk)
        self.btn_max_jerk.pack(side=RIGHT, anchor="n", padx=5, pady=5)

        ##########Custom Command Line##########

        self.lb_command_line = Label(self.middle_panel, text="Command", width=10, bg="#f2f2f2")
        self.lb_command_line.pack(side=LEFT, anchor="n", padx=5, pady=5)
        self.command_line = StringVar()
        self.et_command_line = Entry(self.middle_panel, textvariable=self.command_line, width=80, state="normal")
        self.et_command_line.pack(side=LEFT, anchor="n", padx=2, pady=5)

        self.btn_command = Button(self.middle_panel, text=" Send Command ", bitmap="gray12", width=115, height=15, compound=LEFT, anchor="w", command=self.send_command)
        self.btn_command.pack(side=RIGHT, anchor="n", padx=5, pady=5)

        self.motor_text = ScrolledText(self.lower_panel, state="normal", width=180)
        self.motor_text.bind("<KeyPress>", lambda e: "break")
        self.motor_text.pack(side=LEFT, anchor="n", padx=5)

        self.controller = MotorController(self.motor_text)
        self.profile = os.path.join(os.path.split(os.path.realpath(__file__))[0], "default.conf")

        self.controller.output_to_panel("Note: some of new features are still unfinished.\nAbout acceleration settings, they all in unit/sec^2. \nIf you have problems in using, please contact us\n")

    def send_command(self):
        self.controller.send_cmd(self.command_line.get(), self)

    def set_travel(self):
        self.controller.set_motor(1, self)
    
    def set_step(self):
        self.controller.set_motor(2, self)
    
    def set_direction(self):
        self.controller.set_motor(3, self)
    
    def set_velocity(self):
        self.controller.set_motor(4, self)

    def set_max_printing_acceleration(self):
        pass

    def set_max_traveling_acceleration(self):
        pass

    def set_max_jerk(self):
        pass

    def set_extruder_pid(self):
        pass

    def set_bed_pid(self):
        pass
    
    def connect(self):
        try:
            self.controller.output_to_panel("Start to initialize configuration...\n")
            self.controller.init_config(self.profile, self.port_item.get(), False)
            self.controller.output_to_panel("Start to set boot mode...\n")
            self.controller.set_normal()
        except Exception as e:
            self.controller.output_to_panel("Error, Connecting failed! Details: " + str(e) + "\n")
            return
    
    def disconnect(self):
        self.controller.close_serial()
        self.controller.output_to_panel("Serial port has been closed!\n")

    def reconnect(self):
        self.controller.reconnect_serial()
        self.controller.output_to_panel("Serial port has been reconnected!\n")
    
    def get_motor_status(self):
        self.controller.get_motor_status(self)
        

if __name__ == '__main__':
    master = Tk()
    master.title('Smatto Motor Tool')
    master.iconbitmap(os.path.join(os.path.split(os.path.realpath(__file__))[0], "tool.ico"))
    app = Application(master)
    app.mainloop()






