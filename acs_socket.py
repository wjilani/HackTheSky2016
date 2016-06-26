#!/usr/bin/env python
#
# acs_socket.py - UDP Socket wrapper for ACS network messaging
#
#Written in 2015 by the Advanced Robotic Systems Engineering Laboratory at the
#U.S. Naval Postgraduate School, Monterey, California, USA.
#
#Pursuant to 17 USC 105, this work is not subject to copyright in the
#United States and is in the public domain.
#
#THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
#REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
#AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
#INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
#LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
#OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
#PERFORMANCE OF THIS SOFTWARE.
#

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports
import copy
import netifaces
import random      # NOTE: Only here for debug
import socket
import time

# Message definitions
import acs_messages as messages
import hmacs_pkts

#-----------------------------------------------------------------------
# Support for reliable transport
# NOTE: This is transitional work, and is subject to much change

'''
ABSTRACT RELIABLE PROTOCOL DEFINITION
(for a single remote endpoint and without broadcast case)

<init>
------------------------------
seq = 0                        # first reliable message is 1
ack = NULL                     # highest received seq + 1
send_buf = []
recv_buf = []

<timer>
------------------------------
time = TIME()
for (data, m_seq, m_ack, m_syn, m_rel, p_time) in send_buf
    if time >= p_time + TIMEOUT
        if ack                 # may have new ack (retain old ack if not set)
            m_ack = ack
        if m_ack               # ack guards reliable send before syn finishes
            pkt = MAKEPACKET(data, m_seq, m_ack, m_syn, m_rel)
            SENDPACKET(pkt)
            send_buf[m_seq] = (data, m_seq, m_ack, m_syn, m_rel, time)

send(data, is_rel)
------------------------------
if is_rel
    seq++
m_seq = seq
m_ack = ack
m_syn = FALSE
m_rel = is_rel
if not m_rel and not m_ack     # ack guard overridden for unreliable sends
    m_ack = 1
if m_rel and m_seq == 1        # first reliable send syns, ack guard overridden
    m_ack = 1
    m_syn = True
if m_rel
    send_buf[m_seq] = (data, m_seq, m_ack, m_syn, m_rel, TIME())
if m_ack                       # ack guards reliable send before syn finishes
    pkt = MAKEPACKET(data, m_seq, m_ack, m_syn, m_rel)
    SENDPACKET(pkt)

recv(pkt)
------------------------------
(data, m_seq, m_ack, m_syn, m_rel) = PARSEPACKET(pkt)
if m_syn or not ack            # remote sender may re-syn at any time
    ack = m_seq + 1
    recv_buf = []
if not m_syn and seq + 1 >= m_ack
    send_buf = send_buf[m_ack:]
if m_rel
    if m_seq >= ack            # implements Go-Back-N acknowledgment
        recv_buf[m_seq] = data
    while recv_buf[ack]        # ordered delivery code unused and omitted
        ack++
RETURN(data)

************
LIMITATIONS:
 * There is no enforced ordering of received messages passed up to
   the application. While the receiver will only ACK the highest
   in-order message it has received, any message is allowed through.
   This can be changed by modifying the receive logic in Socket().
 * There are no provisions for broadcast reliability. To do so would
   require additional logic to detect when some-but-not-all receivers
   have not seen a message. A NACK mechanism may also help here.
   Note that the current implementation does not "know" the full set
   of entities in existence, so there is no way to decide that ALL
   entities have received a message. At best we know that NO entities
   that we HAVE heard from HAVEN'T received it.
 * To avoid requiring unicast ACKs, the ground station interprets the
   ACK (.msg_ack) from aircraft as ACKs to it specifically. As a
   consequence, only one ground station can send reliable messages
   to aircraft. Otherwise, there will be competition for the meaning
   of that field, and resulting confusion in the ACKs. There are no
   sanity checks preventing this, however. See the Socket instance
   member _rel_recv_id for how this is "handled" currently.
 * Because there are many aircraft, no periodic messages from the
   ground to individual aircraft, and no explicit ACKs, there is
   no good way to get ACKs from the ground to the air. Therefore,
   there is no support for air-to-ground reliability at this time.
 * Aircraft-to-aircraft reliable messaging is not supported. This is
   probably solved once air-to-ground reliable messaging is solved.
 * Sequence number wrapping is not handled; if an entity gets to
   sending > 2^16 reliable messages during a mission, we'll need to
   fix that.
'''

# Glorified struct for a single message being sent reliably
class ReliableMessage(object):

    def __init__(self, msg, send_params):
        self.msg = msg
        self.send_params = send_params
        self.first_sent = None    # TODO: Eliminate if not used
        self.last_sent = None

    def stamp(self, t=None):
        if not t: t = time.time()
        if not self.first_sent: self.first_sent = t
        self.last_sent = t

# State container and management between self and one remote entity
class ReliableState(object):
    TIME_RESEND = 0.5   # Seconds to wait before resend-eligible

    def __init__(self, init_seq=0, init_ack=None):
        self._send_seq = init_seq  # Highest msg_seq sent to remote
        self._send_buf = {}        # Dict of unACKed sent ReliableMessage's
        self._recv_ack = init_ack  # Highest msg_seq ACKed from remote + 1
        self._recv_buf = {}        # Dict of unACKed received Message's

    # Get current, or next, reliable sequence number
    def getSeq(self, increment=False):
        if increment: self._send_seq += 1
        return self._send_seq

    # Get highest seqnum we've ACKed + 1
    # NOTE: may be None if not init'd
    def getAck(self):
        return self._recv_ack

    # Reset receiver state (do during first send, first recv, or after a syn)
    def resetRecv(self, ack=1):
        self._recv_ack = ack   # May cause sender to drop unACKed messages
        self._recv_buf = {}    # May lose some OOO messages

    # Add a (sent) Message, maintaining msg_seq order
    def addSent(self, msg, send_params, t=None):
        if not isinstance(msg, messages.Message):
            raise Exception("Parameter is not a Message")

        # Create struct for Message and mark timestamp
        relmsg = ReliableMessage(msg, send_params)
        relmsg.stamp(t)

        # Add to send buffer
        self._send_buf[msg.msg_seq] = relmsg

    # Return number of unACKed messages in buffer
    def getUnackCount(self):
        return len(self._send_buf)

    # Return list of timed out ReliableMessage's in msg_seq order,
    # with our updated ACK number
    # Optionally specify time, otherwise use current time
    def getTimedOut(self, t=None):
        if not t: t = time.time()
        msgs = [self._send_buf[s] for s in sorted(self._send_buf) \
                if t >= self._send_buf[s].last_sent + self.TIME_RESEND]
        #for m in msgs:  # NOTE: This looks like a bug (may wreck SYN resends)
        #    m.msg.msg_ack = self._recv_ack
        return msgs

    # Process (filter) messages that have been acknowledged
    def processAck(self, msg):
        ack = msg.msg_ack
        if self._send_seq + 1 < ack:
            # Make sure this ACK is possible (could be off after a reset)
            return
        self._send_buf = {k:v for k,v in self._send_buf.items() if k >= ack}

    # Update internal ACK based on in-order messages seen, and return
    # any messages that are now in-order.
    # NOTE: It's up to the transport logic to decide whether to enforce
    # any ordering; we don't care if received messages are delivered OOO
    def processSeq(self, msg):
        seq = msg.msg_seq

        # If this sequence number is ahead of what we've ACKed, add to list,
        # making sure list is sorted and nothing in list is below our ACK.
        if seq >= self._recv_ack:
            self._recv_buf[seq] = msg

        # Advance ACK num and clear recv buffer for in-order messages
        acked_msgs = []
        while self._recv_ack in self._recv_buf:
            acked_msgs.append(self._recv_buf[self._recv_ack])
            del self._recv_buf[self._recv_ack]
            self._recv_ack += 1

        return acked_msgs

#-----------------------------------------------------------------------
# Socket class

class Socket():
    # Constant Entity IDs
    ID_BCAST_ALL = 0xff

    # Must provide EITHER device OR (my_ip AND bcast_ip). 
    # If you provide both, ip's are used
    # Mapping IDs to IPs: mapped_ids[id] = ip_address
    def __init__(self, my_id, udp_port, device=None, 
                 my_ip=None, bcast_ip=None, mapped_ids=None,
                 send_only=False, promisc=False, bcast_bind=False):
        # Instance variables
        self._port = udp_port		# UDP port for send/recv
        self._id = my_id		# Local entity ID (0..255 currently)
        self._subswarm = 0		# Local entity subswarm ID
        self._idmap = mapped_ids	# Mapping of IDs to IPs
        self._ip = my_ip		# Local entity IP address
        self._bcast = bcast_ip		# Broadcast IP address
        self._sock = None		# UDP socket
        self._sendonly = send_only	# Don't bind a port
        self._promisc = promisc 	# Receive packets not for me

        # Reliability variables
        self._rel_states = {}		# Dict of ReliableState's, by entity ID
        self._rel_recv_id = 0		# Source ID of last-received reliable msg
        
        # If user did not specify both addresses,
        # Attempt to look up network device addressing information
        '''if not my_ip or not bcast_ip:
            try:
                self._ip = netifaces.ifaddresses(device)[2][0]['addr']
                self._bcast = netifaces.ifaddresses(device)[2][0]['broadcast']
            except Exception:
                raise Exception("Couldn't establish IP addressing information")
        '''
        # Build the socket
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            if send_only:
                # Allow a socket that can send but not receive
                # NOTE: may no longer be necessary in light of SO_REUSEADDR
                pass
            elif my_ip and bcast_ip:
                # For now, assume that manual IP specification implies
                #  an environment where we cannot bind to 0.0.0.0 (e.g. SITL)
                # TODO: This is sure to bite us later on, but unclear where
                self._sock.bind((self._ip, self._port))
            elif bcast_bind:
                # Allow socket to bind to the device broadcast address.
                # It will hear messages destined to that address, but NOT
                # unicast messages.
                self._sock.bind((self._bcast, self._port))
            else:
                self._sock.bind(('', self._port))
        except Exception as ex:
            raise Exception("Socket init error: " + str(ex))

    # Get and set subswarm ID in Pythonic property style
    # NOTE: Apparently must reference internally by property name,
    #  *not* by internal variable name. Not clear why.
    @property
    def subswarm(self):
        return self._subswarm
    @subswarm.setter
    def subswarm(self, value):
        self._subswarm = value

    # Look up particular ReliableState by ID
    def _getRelState(self, remote_id):
        if remote_id not in self._rel_states:
            self._rel_states[remote_id] = ReliableState()
        return self._rel_states[remote_id]

    # Get the IP I use to talk on the network
    def getIP(self):
        return self._ip

    # Return list of IDs for which we have buffered sent messages still
    def getUnackDstList(self):
        return [k for k,v in self._rel_states if v.getUnackCount() > 0]

    # Do actual serialization and send
    def _send(self, msg, dst_ip): 
        try:
            ''' DEBUG CODE
            if msg.msg_fl_rel:
                print "DEBUG: send reliable message #%u" % msg.msg_seq

            if not random.choice([True, False]):
                print '(simulated lost packet)'
                return True
            '''
            # Pack message into byte string
            data = msg.serialize()
	    # (HackTheSky-Team BIGMAC) Add HMAC authentication to all sent
            # messages.
	    hm = hmacs_pkts.HmacMsg(msg.msg_secs, data, "/opt/keys.txt")
	    (enmsg, hmValue) = hm.doHmacCrypto(disable=False)
            data = enmsg + hmValue
            # Send it, return number of bytes sent (per sendto())
	    dst_ip = "192.168.10.255"
	    self._port = 5554
            return self._sock.sendto(data, (dst_ip, self._port))

        # Print any exception for user's awareness
        except Exception as ex:
            print("ACS send: " + str(ex))
            return False

    # Resend any timed out messages
    def _resend(self, t=None):
        if not t: t = time.time()

        # Iterate through per-remote states
        # TODO: May want to restrict to remote(s) from which we've seen data
        for dst,state in self._rel_states.items():
            ack = state.getAck()

            # Iterate through buffered messages in each state
            for relmsg in state.getTimedOut(t):
                # If state's ACK is set, update the message's ACK
                if ack is not None:
                    relmsg.msg.msg_ack = ack
                # If message's ACK is not set, don't send
                if relmsg.msg.msg_ack is None:
                    continue

                # Resend the message and update timestamp
                self._send(relmsg.msg, *relmsg.send_params)
                relmsg.stamp(t)

    # like send() below, but no protocol and no header manipulation
    def sendExact(self, msg):
        if not msg or not isinstance(msg, messages.Message):
            raise Exception("Parameter is not a Message")
        return self._send(msg, self._bcast)

    # 'msg' must be a valid Message subclass
    def send(self, given_msg):
        if not given_msg or not isinstance(given_msg, messages.Message):
            raise Exception("Parameter is not a Message")

        # Make a copy so we can manipulate it and not affect the original
        msg = copy.copy(given_msg)

        # Enforce sender ID and subswarm ID
        msg.msg_src = self._id
        msg.msg_sub = self.subswarm

        # If time wasn't set, set it to zero
        if msg.msg_secs is None or msg.msg_nsecs is None:
            msg.msg_secs = 0
            msg.msg_nsecs = 0

        # If sending to a device with a known ID->IP mapping, use it;
        # otherwise use IP broadcast
        #dst_ip = self._bcast
	dst_ip = '192.168.10.255'
        if self._idmap and (msg.msg_dst != Socket.ID_BCAST_ALL) \
                       and (msg.msg_dst in self._idmap):
            dst_ip = self._idmap[msg.msg_dst]

        ### BEGIN RELIABILITY SECTION ###

        # Can not send reliably to multi-entity destinations
        if msg.msg_dst & ~messages.SUBSWARM_MASK == ~messages.SUBSWARM_MASK:
            msg.msg_fl_rel = False

        # Look up ReliableState to the destination
        if msg.msg_dst == Socket.ID_BCAST_ALL:
            # State of the last sender to send reliably to us
            # NOTE: this is part of the broadcast ACK hack.
            state = self._getRelState(self._rel_recv_id)
        else:
            state = self._getRelState(msg.msg_dst)

        # Set reliability fields
        msg.msg_seq = state.getSeq(msg.msg_fl_rel)
        msg.msg_ack = state.getAck()
        msg.msg_fl_syn = False

        # Handle cases where ACK is not set for this destination
        # NOTE: we use msg_ack == None to block reliable messages from
        # being sent until we have sync with remote end
        if not msg.msg_fl_rel and msg.msg_ack is None:
            # Unreliable messages get ACK==1, which acks zero messages
            msg.msg_ack = 1
        if msg.msg_fl_rel and msg.msg_seq == 1:
            # The first reliable message is a SYN and gets a zero ACK,
            # EVEN if we've seen traffic already (remote side must sync
            # its ACK to our SEQ)
            msg.msg_ack = 1
            msg.msg_fl_syn = True

        # Add reliable messages to ReliableState send buffer
        if msg.msg_fl_rel:
            state.addSent(msg, [dst_ip])

        ### END RELIABILITY SECTION ###
        # If ACK is set, send the message
        if msg.msg_ack is not None:
            return self._send(msg, dst_ip)
        return None

    # Return values:
    #  - <Object> - Valid received message object
    #  - False    - Saw a message, but not one for us (might be more)
    #  - None     - No messages available
    def recv(self, buffsize=1024):
        if self._sendonly:
            raise Exception("Attempted to receive on send-only socket")

        if not buffsize:
            raise Exception("Invalid receive buffer size")

        # Check if there are any newly received messages
        # NOTE: Currently, if this fails then we return None. If we want
        # ordered delivery, we might want to fall into the logic below,
        # and return the next-available in-order message, if there are any.
        # Another option is to always return a list of new in-order messages
        # and leave this logic as-is.
        try:
            data, (ip, port) = self._sock.recvfrom(buffsize, socket.MSG_DONTWAIT)
            # Mostly likely due to no packets being available
            if not data:
                return None
        except Exception as ex:
            # Mostly likely due to no packets being available
            return None
        
        # If anything goes wrong below, return False so caller knows
        #  there may be more packets to receive
        try:
            # Parse message
            msg = messages.Message.parse(data)

            # Is it from us? If so, ignore ourselves
            if msg.msg_src == self._id:
                return False
            
            # Is it meant for us?
            if not (msg.msg_dst == self._id or \
                    msg.msg_dst == Socket.ID_BCAST_ALL or \
                    (msg.msg_dst & ~messages.SUBSWARM_MASK == ~messages.SUBSWARM_MASK and \
                     msg.msg_dst & messages.SUBSWARM_MASK == self.subswarm)) \
               and not self._promisc:
                return False
            
            # Add source IP and port, just in case someone wants them
            msg.msg_src_ip = ip
            msg.msg_src_port = port

            ### BEGIN RELIABILITY SECTION ###

            # Process seq and ack numbers from this message
            state = self._getRelState(msg.msg_src)
            if msg.msg_fl_syn or state.getAck() is None:
                # Reset receiver state based on sender information
                state.resetRecv(msg.msg_seq + 1)
            if not msg.msg_fl_syn:
                # If SYN flag is set, sender's ACK is invalid
                # NOTE: This code fires regardless if the message was unicast
                # or broadcast, reliable or unreliable. This is both what allows
                # aerial broadcasts to ACK ground commands, and part of the
                # reason we can't perform air-to-ground reliable messaging.
                state.processAck(msg)
            if msg.msg_fl_rel and msg.msg_dst == self._id:
                # Process reliable, unicast message for us
                # NOTE: This function returns a list of new in-order messages;
                # can use this if we want in-order delivery
                state.processSeq(msg)
                # Note the sender ID
                # NOTE: This is a hack so aircraft can bcast ACKs to ground
                self._rel_recv_id = msg.msg_src

            # Check for timed out messages and resend them
            # NOTE: This should happen in a timer-driven function somewhere
            # else, but conveniently this function gets called pretty often
            # so we're making use of it until we redesign this library.
            self._resend()

            # NOTE: We can implement NACK by checking the msg.msg_seq against
            # our state. (!msg_fl_rel && msg_seq >= ACK) or (msg_fl_rel &&
            # msg_seq > ACK) means we're missing a reliable message.

            ### END RELIABILITY SECTION ###

            # (HackTheSky-Team BigMac) Unwrap HMAC authentication added above
            # in _sendto()
            hm = hmacs_pkts.HmacMsg(msg.msg_secs, data, "/opt/keys.txt")
            data = hm.checkRecvCmd(data)
            if data is None:
                print "HMAC unwrapping failed!"
                raise Exception('HMAC unwrapping failed!')
                return None
            
            return msg
            
        # Any other unhandled conditions are printed for our awareness
        except Exception as ex:
            print("ACS recv: " + str(ex))
            return False

