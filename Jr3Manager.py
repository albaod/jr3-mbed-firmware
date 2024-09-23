import serial
import threading
import time
from enum import Enum
from queue import Queue, Empty

TIMEOUT_READ = 5.0

class JR3Command(Enum):
    ACK = 1
    START = 2
    STOP = 3
    ZERO_OFFS = 4
    SET_FILTER = 5
    GET_STATE = 6
    GET_FS = 7
    RESET = 8
    READ = 9
    BOOTUP = 10

class JR3State(Enum):
    READY = 0  
    NOT_INITIALIZED = 1

class SerialMsg:
    def __init__(self, op=0, data=None):
        self.op = op
        self.data = data if data else bytearray()
        self.size = len(self.data)

def build_message(msg):
    buffer = bytearray()
    buffer.extend(b'<%02d' % msg.op)
    if msg.size > 0:
        buffer.extend(msg.data)
    buffer.extend(b'>')
    return buffer

class JR3Manager:
    def __init__(self, channel, baudrate, timeout=TIMEOUT_READ):
        self._ser = serial.Serial(channel, baudrate)
        self._data_queue = Queue(maxsize=1)
        self._running = True
        self._thread = threading.Thread(target=self._do_work)
        self._thread.start()
        self._fs_factors = None
        self._last_state_time = None
        self._state = JR3State.NOT_INITIALIZED
        self._forces = None
        self._torques = None
        self._last_read = None
        self._framecounter = None
        self._timeout_read = timeout
        self._last_command = None
        self._read_count = 0

    def __del__(self):
        self._running = False
        self._thread.join()
        self._ser.close()

    def start(self, fc, period):
        if not self.get_fs()[0]:
            return False, self._state

        start = time.time()
        while time.time() - start < 0.025: 
            if hasattr(self, '_last_ack_time') and self._last_ack_time - start > 0:
                return True, self._state
            time.sleep(0.001)

        msg_out = self._callgenerator_send(fc, period, JR3Command.START.value)
        return msg_out, self._state

    def stop(self):
        success = self._callgenerator_send(None, None, JR3Command.STOP.value)
        if success:
            self._send_message(SerialMsg(JR3Command.STOP.value))
            return True, self._state
        return False, self._state

    def zero_offs(self):
        success = self._callgenerator_send(None, None, JR3Command.ZERO_OFFS.value)
        return success, self._state

    def set_filter(self, fc):
        success = self._callgenerator_send(fc, None, JR3Command.SET_FILTER.value)
        return success and self._state == JR3State.READY, self._state

    def get_state(self):
        if self._state is None or self._last_state_time is None or (time.time() - self._last_state_time > 5):
            if not self._callgenerator_send(None, None, JR3Command.GET_STATE.value):
                return False, None
        return True, self._state

    def get_fs(self):
        if self._fs_factors is None:
            if not self._callgenerator_send(None, None, JR3Command.GET_FS.value):
                return False, None
        return True, self._fs_factors, self._state

    def reset(self):
        return self._callgenerator_send(None, None, JR3Command.RESET.value) and self._state == JR3State.READY

    def read(self):
        if not self._last_read is None and time.time() - self._last_read < 0.1:
            return True, self._forces, self._torques, self._framecounter
        return False, None, None, 0

    def _do_work(self):
        while self._running:
            self._read_message()
            time.sleep(0.001)

    def _callgenerator_send(self, fc, period, command):
        self._last_command = command
        msg_out = self._generador_msg(fc, period, command)
        if not msg_out:
            return False
        try:
            self._data_queue.get(block=False)
        except Empty:
            pass
        if not self._send_message(msg_out):
            return False
        self._last_ack_time = time.time()
        timeout = 5  # Timeout de 5 segundos
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                msg = self._data_queue.get(timeout=0.01)  # Intentar obtener mensaje con timeout de 0.01s

                if msg.op != JR3Command.ACK.value or msg.size == 0:
                    continue

                self._state = JR3State(msg.data[0])
                self._last_state_time = self._last_ack_time

                if command == JR3Command.GET_FS.value and msg.size == 13: 
                    self._fs_factors = self._process_fs_factors(msg.data)
                return True
            except Empty:
                continue
        return False

    def _generador_msg(self, frecuencia, periodo, command):
        if command == JR3Command.START.value:
            frecuencia_bytes = frecuencia.to_bytes(2, 'little')
            periodo_bytes = periodo.to_bytes(4, 'little')
            data = frecuencia_bytes + periodo_bytes
            return SerialMsg(command, data)
        elif command == JR3Command.SET_FILTER.value:
            frecuencia_bytes = frecuencia.to_bytes(2, 'little')
            data = frecuencia_bytes
            return SerialMsg(command, data)
        else:
            data = bytearray()
            return SerialMsg(command, data)

    def _send_message(self, msg, buffer=None):
        try:
            message = build_message(msg)
            self._ser.write(message)
            if buffer is not None:
                buffer.extend(message)
            return True
        except serial.SerialException as e:
            return False

    def _process_fs_factors(self, data):
        factors = []
        for i in range(1, len(data), 2):
            factors.append(int.from_bytes(data[i:i+2], 'little'))
        return factors

    def _parse_message(self, msg):
        if msg.op == JR3Command.GET_FS.value:
            if msg.size > 0:
                data_to_process = msg.data[2:]  
                self._fs_factors = self._process_fs_factors(data_to_process)
            else:
                self._fs_factors = []
        elif msg.op == JR3Command.READ.value:
            if msg.size == 14: 
                if self._fs_factors is not None and len(self._fs_factors) >= 6:
                    self._forces = [int.from_bytes(msg.data[2*i:2*i+2], 'little', signed=True) / self._fs_factors[i] for i in range(0, 3)]
                    self._torques = [int.from_bytes(msg.data[2*i:2*i+2], 'little', signed=True) / (self._fs_factors[i] * 10) for i in range(3, 6)]
                    self._framecounter = msg.data[6]
                    self._last_read = time.time()
                else:
                    self._forces = None
                    self._torques = None
        self._read_count += 1

    def _read_message(self):
        buffer_in = self._ser.read_until(b'>')
        cleaned_msgs = JR3Manager._clean_message(buffer_in)

        for buffer in cleaned_msgs:
            msg = SerialMsg()
            msg.size = 0

            start_idx = buffer.find(b'<')
            end_idx = buffer.find(b'>')

            if start_idx == -1 or end_idx == -1 or start_idx > end_idx:
                return False

            buffer = buffer[start_idx + 1:end_idx]

            if len(buffer) < 2:
                return False

            try:
                opcode_str = buffer[:2].decode()
                msg.op = int(opcode_str)
                msg.data = buffer[2:]
                msg.size = len(msg.data)

                if msg.op == JR3Command.ACK.value:
                    self._data_queue.put(msg)
                    self._last_ack_time = time.time()
                elif msg.op == JR3Command.BOOTUP.value:
                    pass
                elif msg.op == JR3Command.READ.value:
                    self._parse_message(msg)
                else:
                    pass
            except Exception as e:
                return False

        return True

    def _clean_message(buffer):
        out = []
        start_idx = buffer.find(b'<')

        while start_idx != -1:
            end_idx = buffer.find(b'>', start_idx)
            if end_idx != -1:
                out.append(buffer[start_idx:end_idx + 1])
                start_idx = buffer.find(b'<', end_idx)
            else:
                break

        return out
