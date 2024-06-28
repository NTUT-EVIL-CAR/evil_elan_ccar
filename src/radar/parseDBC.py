import os
import re
import copy
import json

class DBC:
    def __init__(self):
        self.dbc_json = {}
        self.bus = {}
        self.signals = {}
        self.node = {}
        filePath = "/home/evil/Desktop/elan_resource/Peter/cubtek_B122_035v1.dbc"
        if filePath:
            # print(filePath)
            with open(filePath, "r") as f:  # 打开文件
                self.data = f.read()  # 读取文件
                # print(data)

    def read_bus(self):
        read_bus = re.findall("BO_ (.+?):", self.data)
        # print(read_bus)
        print(read_bus)
        for bus in read_bus:
            bus = str(bus).split()
            print(bus[1])
            dick = {"bus": bus[1]}
            self.dbc_json.update(dick)
        return self.dbc_json

    def read_signals(self):
        signal = re.findall("SG_ (.+?):", self.data)
        for sig in signal:
            i = str(sig).split()
            if len(i) == 1:
                self.signals = {"signal": i[0]}
                print(self.signals)
            else:
                pass

    def read_nodes(self):
        pattern = re.compile(r"BO_ (.*?) (.*?): (.*?) (.*)")
        nodes = re.findall(pattern, self.data)
        node_list = []
        for node in nodes:
            #  ('788', 'CCU_01', '8', 'CGW')
            node_list.append(node[3])
        print(set(node_list))
        return set(node_list)

    def bdc_dick(self):
        for i in self.read_nodes():
            dick_i = {i: []}
            self.dbc_json.update(dick_i)
        print(self.dbc_json)
        for a in self.read_bus():
            dick_a = {a: []}
            self.dbc_json.update()


if __name__ == '__main__':
    dbc = DBC()
    print(dbc.read_signals())
    print(dbc.read_bus())
    print(dbc.read_nodes())
    print(dbc.bdc_dick())