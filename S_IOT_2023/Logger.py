import os, sys, traceback

# See https://stackoverflow.com/a/57008707
class Logger(object):
    def __init__(self, filename):
        # Si le dossier "logger_output" n'existe pas on le crée
        if "logger_output" not in os.listdir(os.getcwd()):
            os.mkdir(os.path.join(os.getcwd(), "logger_output"))
        self.file = open(os.path.join("logger_output",filename), 'w')
        self.stdout = sys.stdout

    def __enter__(self):
        sys.stdout = self

    def __exit__(self, exc_type, exc_value, tb):
        sys.stdout = self.stdout
        if exc_type is not None:
            self.file.write(traceback.format_exc())
        self.file.close()

    def write(self, data):
        self.file.write(data)
        self.stdout.write(data)

    def flush(self):
        self.file.flush()
        self.stdout.flush() 