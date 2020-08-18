import pickle
import struct
import re
import os

file_index=[]
file_dic={}
def ascii_bin(file_path_in,lindex,rindex,file_path_out):
    f1=open(file_path_in,'rb')
    f2=open(file_path_out,'wb+')
    line=f1.readline()
#    for i in range(lindex,rindex-1,2):
    i=0
    while i < rindex-1:
        if(i % 4224 == 0):
            i=i+16     
        if( 48<=line[i]<=57):
            c=line[i]-48   
        else:
            c=line[i]-87
            
        if( 48<=line[i+1]<=57):
            d=line[i+1]-48   
        else:
            d=line[i+1]-87
        bin_data1 = c
        bin_data2 = d

        bin_data  = (bin_data1<<4) | (bin_data2)

        try:
            bin_out   = struct.pack('B',bin_data)
        except:
            print('the error index is '+str(i)+'\n')
        i=i+2
        f2.write(bin_out)

    f1.close()
    f2.close()


#####################################
#                                   #
# the function for get time stamp   #
#                                   #
# year_month_day_hour-minute-second #
#                                   #
#####################################

def timestamp(file_path):

    fin=open(file_path,'r')
    line=fin.readline()
    target='0000000000000000'
#    print(str(int(line[56]+line[57],16)))
    result = re.finditer(target,line)

    for i in result:
#        print(i.span()[0])
        if((i.span()[0])%4224 ==0):
            file_dic[i.span()[0]]='20'+str(int(line[i.span()[0]+56]+line[i.span()[0]+57],16))+'_'\
                                   +str(int(line[i.span()[0]+40]+line[i.span()[0]+41],16))+'_'\
                                   +str(int(line[i.span()[0]+48]+line[i.span()[0]+49],16))+'_'\
                                   +str(int(line[i.span()[0]+16]+line[i.span()[0]+17],16))+'-'\
                                   +str(int(line[i.span()[0]+24]+line[i.span()[0]+25],16))+'-'\
                                   +str(int(line[i.span()[0]+32]+line[i.span()[0]+33],16))
#    print(file_dic)
    fin.close()



############################
#
# cut file depending file_dic
#
############################   
def file_cut(file_path_in,length):
    cut_index=[]
    file_path_out=os.getcwd()
    k=file_dic.keys()
    for i in k:
        cut_index.append(i)
#    print(cut_index)

    for i in range(0,len(cut_index)):
        file_name=file_dic[cut_index[i]]
        if(i==len(cut_index)-1):
            lindex=cut_index[i]+64
            rindex=length
#            print(lindex)
#            print(rindex)
        else:
            lindex=cut_index[i]+64
            rindex=cut_index[i+1]
#            print(lindex)
#            print(rindex)

        file_path_out=file_path_out+'\\'+file_name+".DAT"
#        print(file_path_out)
        ascii_bin(file_path_in,lindex,rindex,file_path_out)
    


    
file_path = input("Enter your filepath: ")
#file_path='D:\OneDrive - mail.hfut.edu.cn\程式安装包\SSCOM_v5.13.1\ReceivedTofile-COM4-2020_8_17_19-31-13.DAT'
fin=open(file_path,'r')
#fout='D:\OneDrive - mail.hfut.edu.cn\程式安装包\SSCOM_v5.13.1\out.DAT'
file_lenth=len(fin.readline())
fin.close()

timestamp(file_path)

file_cut(file_path,file_lenth)

