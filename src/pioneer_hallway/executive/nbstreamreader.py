from threading import Thread
from Queue import Queue, Empty

class NonBlockingStreamReader(object):

    def __init__(self, stream):
        '''
        stream: the stream to read from.
                Usually a process' stdout or stderr.
        '''

        self.stream = stream
        self.queue = Queue()

        def _populateQueue(stream, queue):
            '''
            Collect lines from 'stream' and put them in 'quque'.
            '''

            while True:
                line = stream.readline().rstrip()
                if line:
                    queue.put(line)
                else:
                    raise UnexpectedEndOfStream

        self.thread = Thread(target = _populateQueue,
                args = (self.stream, self.queue))
        self.thread.daemon = True
        self.thread.start() #start collecting lines from the stream

    def readline(self, timeout = None):
        try:
            return self.queue.get(block = timeout is not None,
                    timeout = timeout)
        except Empty:
            return None

class UnexpectedEndOfStream(Exception): pass
