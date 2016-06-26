#!/usr/bin/env python

'''
Team BIGMAC @Hackthesky
@author wahobbit@gmail.com
Twitter @jilaniwahab
'''

import time, hmac, hashlib, binascii
import subprocess

class HmacMsg():

    def __init__(self, msgTime=None, msg='', keyFile=None):
        self.time    = msgTime
        self.msg     = msg
        self.keyFile = keyFile
        self.shaKey  = ''
        return

    def encryptMsg(self, decrypt=False):
        '''
        Encrypt msg prior to being sent for HMAC
        '''
        #Check to see if this is correct
        enkey = self.sha1File()
	self.msg = binascii.hexlify(self.msg)
	print "self.msg = " + self.msg
        p1 = subprocess.Popen(["echo", self.msg], stdout=subprocess.PIPE)
        cmd = ["openssl", "enc", "-aes-128-cbc", "-a", "-salt", "-pass", "pass:"+enkey]
        if decrypt == False:
            p2 = subprocess.Popen(cmd, stdin=p1.stdout, stdout=subprocess.PIPE)
        else:
            # Decrypt!
            cmd.append("-d")
            p2 = subprocess.Popen(cmd, stdin=p1.stdout, stdout=subprocess.PIPE)
        p1.stdout.close()
        output=p2.communicate()[0]
        return output.strip()


    def returnHmac(self, hmacKey):
        '''
        Return a HMAC of a string
        '''
        enmsg = self.encryptMsg(decrypt=False)
        hm = hmac.new(hmacKey, enmsg, digestmod=hashlib.sha1)
        return (enmsg, hm.hexdigest())

    def sha1File(self):
        '''
        Hash a CSV and return the hex
        '''
        # Read file as a blob and hash it
        f    = open(self.keyFile, 'rb')
        blob = f.read()
        m    = hashlib.sha1()
        m.update(blob)
        return m.hexdigest()

    def checkRecvCmd(self, msg):
        '''
        Check the received command, received MAC address against the local /tmp/foo.txt
        '''
        hexData     = binascii.hexlify(msg)
        bin_fmt     = '20s'
	hval        = hexData[-20:]
        realData    = hexData[:-20]
        (realData,) = struct.unpack(bin_fmt, realData)

        self.shaKey = self.sha1File()
        key    = self.shaKey + self.time

        hm = hmac.new(key, realData, digestmod=hashlib.sha1)
	if hm.hexdigest()[20:] != hval:
            raise Exception("HMAC unwrapping failed!")
        else:
            self.msg = realData
            decmsg = encryptMsg(decrypt=True)
            return decmsg

    def doHmacCrypto(self, disable=False):
        '''
        Do the actual HMAC and SHA1 for the message
        '''
	if disable == False:
       	    shaKey = self.sha1File()
            key    = shaKey + str(self.time)
            (enmsg, hmsg) = self.returnHmac(key)
	    hmsg = hmsg[0:20]
	    print "HMAC: " + hmsg
	    print "doHmacCrypto: Before returning"
	    return (enmsg, hmsg)
	else:
	    # Disable Crypto!
	    return (self.msg, self.msg)
	
def main():
    '''
    Do all the things
    '''
    hm = HmacMsg(str(int(time.time())), "Hello World", "/opt/keys.txt")
    (enmsg, hmsg) = hm.doHmacCrypto(disable=True)
    print "Encrypted Message: " + enmsg
    print "First 20 HMAC Hexdigest  : " + hmsg
    return True


if __name__ == "__main__":
    main()


