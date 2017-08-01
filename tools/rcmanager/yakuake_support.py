
import subprocess
import shlex

class ProcessHandler():
    def __init__(self):
        proc = subprocess.Popen(shlex.split("yakuake"), shell=False)
        self.tabTitleToSessionId = dict()
        self.sessionIdToTabTitle = dict()

    def start_process_in_new_session(self, tabTitle=None, command=None):
        try:
            ret, sessionId = self.add_session(tabTitle)
            proc = subprocess.Popen(
                shlex.split('qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommandInTerminal ' \
                            + str(sessionId) + ' "' + command + '"'), shell=False)
            sessionProcessId = self.get_session_process_id(tabTitle)
            foregroundProcessId = sessionProcessId

            while foregroundProcessId == sessionProcessId:
                foregroundProcessId = self.get_foreground_process_id(tabTitle)
        except Exception, e:
            raise e
        return (tabTitle, foregroundProcessId)

    def start_process_in_existing_session(self, tabTitle=None, command=None):
        if tabTitle in self.tabTitleToSessionId:
            sessionId = int(self.tabTitleToSessionId[tabTitle])
            proc = subprocess.Popen(
                shlex.split('qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.isSessionClosable ' \
                            + str(sessionId)), stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=False)
            (isSessionExisting, error) = proc.communicate()

            if isSessionExisting == "true\n":
                try:
                    proc = subprocess.Popen(
                        shlex.split('qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommandInTerminal ' \
                                    + str(sessionId) + ' "' + command + '"'), shell=False)
                    sessionProcessId = self.get_session_process_id(tabTitle)
                    foregroundProcessId = sessionProcessId
                    while foregroundProcessId == sessionProcessId:
                        foregroundProcessId = self.get_foreground_process_id(tabTitle)
                except Exception, e:
                    raise e
                return (tabTitle, foregroundProcessId)
        ret = self.start_process_in_new_session(tabTitle, command)
        return ret

    def stop_process_in_session(self, tabTitle=None):
        try:
            processId = self.get_foreground_process_id(tabTitle)
            proc = subprocess.Popen(shlex.split('kill -9 ' + str(processId)), shell=False)
        except Exception, e:
            raise e

    def add_session(self, tabTitle):
        try:
            proc = subprocess.Popen(shlex.split('qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession'), \
                                    stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=False)
            (sessionId, error) = proc.communicate()
            proc = subprocess.Popen(shlex.split('qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle ' + \
                                                str(sessionId) + ' "' + tabTitle + '"'), shell=False)
            self.tabTitleToSessionId[tabTitle] = sessionId
            self.sessionIdToTabTitle[sessionId] = tabTitle
        except Exception, e:
            raise e
        return (tabTitle, sessionId)

    def close_session(self, tabTitle):
        try:
            proc = subprocess.Popen(shlex.split('qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.removeSession ' \
                                                + str(self.tabTitleToSessionId[tabTitle])), shell=False)
        except Exception, e:
            raise e

    def get_foreground_process_id(self, tabTitle):
        sessionIdPlusOne = int(self.tabTitleToSessionId[tabTitle]) + 1
        proc = subprocess.Popen(shlex.split('qdbus org.kde.yakuake /Sessions/' + str(
            sessionIdPlusOne) + ' org.kde.konsole.Session.foregroundProcessId'), stderr=subprocess.PIPE,
                                stdout=subprocess.PIPE, shell=False)
        (processId, error) = proc.communicate()
        return processId

    def get_session_process_id(self, tabTitle):
        sessionIdPlusOne = int(self.tabTitleToSessionId[tabTitle]) + 1
        proc = subprocess.Popen(shlex.split('qdbus org.kde.yakuake /Sessions/' + str(
            sessionIdPlusOne) + ' org.kde.konsole.Session.processId'), stderr=subprocess.PIPE,
                                stdout=subprocess.PIPE, shell=False)
        (processId, error) = proc.communicate()
        return processId

if __name__ == '__main__':
    processHandler = ProcessHandler()
