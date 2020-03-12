import time

class BTNode(object):
    status = ["RUNNING","SUCCESS","FAILURE","IDLE"]
    def __init__(self,name):
        self._name = name
        self._status = "IDLE"

    def Update(self):
        pass

    def OnTick(self):
        pass

    def _cleanup(self):
        pass
    
    def Reset(self):
        self._status = "IDLE"

    def Cancel(self):
        self.Reset()
        self._cleanup()

    def __str__(self):
        return self._name

    def getName(self,stack):
        print(self._name)
        return self._name

class Action(BTNode):
    '''
    Need to implement Update, Execute, _cleanup and override Initialization
    '''
    def __init__(self,name):
        self._name = name # Using Name as unique ID for explanation and visualization

    def Execute(self):
        # TOBE Implemented
        pass

    def Update(self):
        # TOBE Implemented
        return self._status

    def OnTick(self):
        self.Execute()
        result = self.Update()
        return result

    def _cleanup(self):
        # TOBE Implemented
        pass

class ConditionNode(BTNode):
    status = ["SUCCESS","FAILURE","IDLE"]
    def Cancel(self):
        self.Reset()

    def Update(self):
        # TO BE Implemented
        return self._status
    
    def OnTick(self):
        return self.Update()

class LogicNode(BTNode):
    """
    Abstract:
    The result of Logic Node is Purely based on child Nodes
    Need to Implement: Excecute
    """
    def __init__(self,name,childs):
        # A tuple Of Child Nodes
        self._childs = tuple(childs)
        super(LogicNode,self).__init__(name)

    def OnTick(self):
        return self.Execute()

    def Execute(self):
        # TO BE Implemented
        pass
    
    def getChildNumber(self):
        return len(self._childs)

    def Reset(self):
        for child in self._childs:
            child.Reset()
        self._status = "IDLE"

    def Cancel(self):
        for child in self._childs:
            child.Cancel()
        self.Reset()

    def getChilds(self):
        return list(self._childs)

    def getName(self,stack):
        print(self._name)
        for child in self._childs:
            print("    "*stack+"+ "),
            child.getName(stack+1)
        return self._name
        
        
class LogicFallback(LogicNode):
    def Execute(self):
        for child in self._childs:
            childret = child.OnTick()
            if childret == "SUCCESS":
                self.Reset()
                return childret
            elif childret == "RUNNING":
                return childret
            elif childret == "IDLE":
                raise Exception("Child Config failed! Child Name: "+str(child)+
                                " Parent Name: "+self._name)
            else:
                continue
        self.Reset()
        return "FAILURE"

class LogicFallbackMem(LogicNode):
    def __init_(self,name,childs):
        self._childs = tuple(childs)
        super(LogicFallbackMem,self).__init__(name)
        self._visited = [0]*len(childs)

    def Execute(self):
        for i in range(sum(self._visited),len(self._childs)):
            childret = self._childs[i].OnTick()
            if childret == "SUCCESS":
                self.Reset()
                return childret
            elif childret == "RUNNING":
                return childret
            elif childret == "IDLE":
                raise Exception("Child Config failed! Child Name: "+str(self._childs[i])+
                                " Parent Name: "+self._name)
            else:
                self._visited[i] = 1
        self.Reset()
        return "FAILURE"

    def Reset(self):
        super(LogicFallbackMem,self).Reset()
        self._visited = [0]*len(self._childs)

class LogicSequential(LogicNode):
    def Execute(self):
        for child in self._childs:
            childret = child.OnTick()
            if childret == "FAILURE":
                self.Reset()
                return childret
            elif childret == "RUNNING":
                return childret
            elif childret == "IDLE":
                raise Exception("Child Config failed! Child Name: "+str(child)+
                                " Parent Name: "+self._name)
        self.Reset()
        return "SUCCESS"

class LogicSequentialMem(LogicNode):
    def __init__(self,name,childs):
        self._childs = tuple(childs)
        super(LogicSequentialMem,self).__init__(name)
        self._visited = [0]*len(childs)

    def Execute(self):
        for i in range(sum(self._visited),len(self._childs)):
            childret = self._childs[i].OnTick()
            if childret == "FAILURE":
                self.Reset()
                return childret
            elif childret == "RUNNING":
                return childret
            elif childret == "IDLE":
                raise Exception("Child Config failed! Child Name: "+str(self._childs[i])+
                                " Parent Name: "+self._name)
            else: 
                self._visited[i] = 1
        self.Reset()
        return "SUCCESS"

    def Reset(self):
        super(LogicSequentialMem,self).Reset()
        self._visited = [0]*len(self._childs)

class LogicParallel(LogicNode):
    def __init__(self,name,childs,thresh):
        super(LogicParallel,self).__init__(name,childs)
        self._thresh = thresh

    def Execute(self):
        rets = []
        for child in self._childs:
            rets.append(child.OnTick())

        if rets.count("SUCCESS") >= self._thresh:
            self.Reset()
            return "SUCCESS"
        elif rets.count("FAILURE">= len(self._childs)-self._thresh):
            self.Reset()
            return "FAILURE"
        elif rets.count("IDLE") > 0:
            raise Exception("Child Config failed! Child Name: "+str(child)+
                                " Parent Name: "+self._name)
        else:
            return "RUNNING"

class Decorator(BTNode):
    def __init__(self,name,child):
        super(Decorator,self).__init__(name)
        self._child = child

    def OnTick(self):
        # TO BE Implemented
        pass  
    
    def Cancel(self):
        self._child.Cancel()
        self._status = "IDLE"

    def Reset(self):
        self._child.Reset()
        self._status = "IDLE"

class InverseDecorator(Decorator):
    def OnTick(self):
        result = self._child.OnTick()
        if result == "FAILURE":
            return "SUCCESS"
        elif result == "SUCCESS":
            return "FAILURE"
        elif result == "IDLE":
            raise Exception("Child Config failed! Child Name: "+str(self._child)+
                                " Parent Name: "+self._name)
        else:
            return "RUNNING"

class TimedDecorator(Decorator):
    def __init__(self,name,child,time):
        super(TimedDecorator,self).__init__(name,child)
        self._time = time
        self._last_time = 0
        self._last_status = "IDLE"

    def OnTick(self):
        # If Child Node return success or fail, return immediately 
        # If Child Node Stuck at running less than time, return running
        # If Child Node Over timed; return fail.
        if self._last_status == "IDLE":
            self._last_time = time.time()
        childret = self._child.OnTick()
        if childret == "SUCCESS" or childret == "FAILURE":
            self.Reset()
            return childret
        elif childret== "IDLE":
            raise Exception("Child Config failed! Child Name: "+str(self._child)+
                            " Parent Name: "+self._name)
        elif time.time() - self._last_time > self._time:
            self.Cancel()
            return "FAILURE"
        else:
            self._last_status = "RUNNING"
            return "RUNNING"

class LoopDecorator(Decorator):
    '''
    Make Sure there are more loop num success
    '''
    def __init__(self,name,child,loop_num):
        super(LoopDecorator,self).__init__(name,child)
        self._loop_num = loop_num
        self._counter = 0

    def OnTick(self):
        childret = self._child.OnTick()
        if childret == "FAILURE":
            self.Reset()
            return childret
        elif childret == "RUNNING":
            return "RUNNING"
        elif childret == "SUCCESS" and self._counter< self._loop_num:
            self._counter += 1
            return "RUNNING"

