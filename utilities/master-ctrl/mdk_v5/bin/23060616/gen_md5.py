#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import os
import sys
import time
import threading
import hashlib
 
def get_file_md5(file_name):
    """
    计算文件的md5
    :param file_name:
    :return:
    """
    m = hashlib.md5()   #创建md5对象
    with open(file_name,'rb') as src_fp:
        while True:
            data = src_fp.read(4096)
            if not data:
                break
            m.update(data)  #更新md5对象
    
    return m.hexdigest()    #返回md5对象
 
 
def get_str_md5(content):
    """
    计算字符串md5
    :param content:
    :return:
    """
    m = hashlib.md5(content) #创建md5对象
    return m.hexdigest()

# file = r'D:\workspace\ax_board\ax_robot\1.0.7\MDK\output\test'

if __name__ == '__main__':
    if (len(sys.argv)) != 2:
        print ("\nusaging: \n\tpython.exe gen_md5.py \'file path\'\n")
    else:
        path_m = sys.argv[1]
        if path_m[-3:] == "bin":
            print ("src:\t" + path_m)
            m_md5 = get_file_md5(path_m)
            if not m_md5:
                print("123456")
            else:
                MD5_flie = path_m[0:-4]+"_md5.txt"
                with open(MD5_flie,'w+') as md5_fp:
                    md5_fp.write(m_md5.upper())
                    print ("dst:\t" + MD5_flie)
                    print ('md5\t'+m_md5.upper())
                    print("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=")
        else:
            for root, dirs, files in os.walk(path_m):
                for file in files:
                    path = os.path.join(root, file)
                    # m_md5 = get_file_md5(r"D:\workspace\ax_board\ax_robot\1.0.7\MDK\output\bsbd_1.0.6-AOB02U1_app1.bin")
                    if path[-3:] == "bin":
                        print ("src:\t" + path)
                        m_md5 = get_file_md5(path)
                        if not m_md5:
                            print("123456")
                        else:
                            MD5_flie = path[0:-4]+"_md5.txt"
                            with open(MD5_flie,'w+') as md5_fp:
                                md5_fp.write(m_md5.upper())
                                print ("dst:\t" + MD5_flie)
                                print ('md5\t'+m_md5.upper())
                                print("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=")
                    