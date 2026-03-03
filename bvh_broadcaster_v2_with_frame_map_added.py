#!/usr/bin/env python
import tf
import math
import numpy
import rospy
import string
import argparse

# Node
class Node:
    def __init__(self, root=False):
        self.name = None #Este atributo almacenara el nombre del nodo, que podria representar una parte del cuerpo (como Hips, LHipJoint, etc.).
        self.channels = [] #Una lista que almacenara los canales de movimiento de ese nodo. 
                            #Los canales suelen ser traslaciones (Xposition, Yposition, Zposition) y 
                            #rotaciones (Xrotation, Yrotation, Zrotation) que determinan como se mueve la parte del cuerpo.
        self.offset = (0,0,0) #Una tupla que almacena las coordenadas de desplazamiento (offset) del nodo en el espacio tridimensional 
                              #respecto a su nodo principal. Esto se inicializa con los valores (0,0,0), que luego son reemplazados al 
                              #leer los datos del archivo BVH.
        self.children = [] #Una lista que contiene los nodos hijos de este nodo. En una estructura esqueletica, un nodo puede tener multiples hijos 
                           #que representan las articulaciones conectadas a el. Por ejemplo, el nodo de Hips puede tener hijos que representen 
                           #las caderas y las piernas.
        self._is_root = root #Un booleano que indica si este nodo es el nodo raiz del esqueleto (por ejemplo, Hips). Se inicializa como False 
                            #por defecto, pero puede ser True si el nodo es designado como raiz.

    def isRoot(self): #Este metodo devuelve el valor de _is_root, lo que indica si el nodo actual es el nodo raiz.
        return self._is_root #TRUE o FALSE

    def isEndSite(self): #Este metodo devuelve True si el nodo no tiene hijos, lo que lo convierte en un End Site (nodo terminal o extremo de la jerarquia).
        return len(self.children)==0 #TRUE o FALSE
    

# BVHReader
class BVHReader:
    def __init__(self, filename):

        self.filename = filename
        # A list of unprocessed tokens (strings)
        self.tokenlist = []
        # The current line number
        self.linenr = 0

        # Root node
        self._root = None
        self._nodestack = []

        # Total number of channels
        self._numchannels = 0

    def onHierarchy(self, root):
        pass

    def onMotion(self, frames, dt):
        pass

    def onFrame(self, values):
        pass

    # read
    def read(self): # 2 SEGUNDA FUNCION QUE LLAMA ----------------------------------------------------------------------------------------------
        """Read the entire file.
        """
        rospy.loginfo("Leendo el archivo entero --------- PASO 2")
        self.fhandle = file(self.filename) 
        rospy.loginfo("Va a leer la Jerarquia ----------- PASO 3 - readHierarchy()")
        self.readHierarchy() #Aqui va a devolver ese _nodestack creo
        self.onHierarchy(self._root)
        self.readMotion()

    # readMotion
    def readMotion(self):
        """Read the motion samples.
        """
        # No more tokens (i.e. end of file)? Then just return 
        try:
            tok = self.token()
        except StopIteration:
            return
        
        if tok!="MOTION":
            raise SyntaxError("Syntax error in line %d: 'MOTION' expected, got '%s' instead"%(self.linenr, tok))

        # Read the number of frames
        tok = self.token()
        if tok!="Frames:":
            raise SyntaxError("Syntax error in line %d: 'Frames:' expected, got '%s' instead"%(self.linenr, tok))

        frames = self.intToken()

        # Read the frame time
        tok = self.token()
        if tok!="Frame":
            raise SyntaxError("Syntax error in line %d: 'Frame Time:' expected, got '%s' instead"%(self.linenr, tok))
        tok = self.token()
        if tok!="Time:":
            raise SyntaxError("Syntax error in line %d: 'Frame Time:' expected, got 'Frame %s' instead"%(self.linenr, tok))

        dt = self.floatToken()

        self.onMotion(frames, dt)

        # Read the channel values
        for i in range(frames):
            s = self.readLine()
            a = s.split()
            if len(a)!=self._numchannels:
                raise SyntaxError("Syntax error in line %d: %d float values expected, got %d instead"%(self.linenr, self._numchannels, len(a)))
            values = map(lambda x: float(x), a)
            self.onFrame(values)


    # readHierarchy
    def readHierarchy(self): # 3 TERCERA FUNCION QUE LLAMA --------------------------------------------------------------------------------------
        """Read the skeleton hierarchy.
        """

        tok = self.token() #Devuelve una palabra de todo el texto de hasta antes de MOTION
        if tok!="HIERARCHY":
            raise SyntaxError("Syntax error in line %d: 'HIERARCHY' expected, got '%s' instead"%(self.linenr, tok))

        tok = self.token()
        if tok!="ROOT":
            raise SyntaxError("Syntax error in line %d: 'ROOT' expected, got '%s' instead"%(self.linenr, tok))

        self._root = Node(root=True) #Una vez que encuentra "ROOT", crea un nuevo nodo de tipo Node y lo marca como el nodo raiz (root=True).
        self._nodestack.append(self._root) #Este nodo se anade a la pila (_nodestack), que es una estructura usada para rastrear la jerarquia 
                                           #de nodos mientras se procesa el archivo.
        self.readNode()

    # readNode
    def readNode(self):
        """Read the data for a node.
        """
    
        # Read the node name (or the word 'Site' if it was a 'End Site'
        # node)
        name = self.token()
        self._nodestack[-1].name = name #se refiere al nodo actual (el ultimo en la pila).
        
        tok = self.token() #Aqui se lee el siguiente token y se espera que sea una llave de apertura {, que indica el comienzo de la definicion del nodo.
        if tok!="{":
            raise SyntaxError("Syntax error in line %d: '{' expected, got '%s' instead"%(self.linenr, tok)) #Indica la linea de ERROR
    
        while 1: #Se inicia un bucle infinito con while 1, que continuara hasta que se rompa explicitamente con un break.
            tok = self.token() #En cada iteracion, se lee un nuevo token.
            if tok=="OFFSET": #A
                rospy.loginfo("Entro a OFFSET *********")
                x = self.floatToken()
                y = self.floatToken()
                z = self.floatToken()
                self._nodestack[-1].offset = (x,y,z)
            elif tok=="CHANNELS": #B Si el token es "CHANNELS", significa que se van a leer los canales de animacion del nodo, que podrian incluir informacion sobre la rotacion y posicion del nodo.
                n = self.intToken() #Aqui se lee un numero entero n, que indica cuantos canales tiene este nodo.
                channels = [] #Se crea una lista vacia channels.
                for i in range(n): 
                    tok = self.token() #Se leen los nombres de los canales de ANIMACION
                    if tok not in ["Xposition", "Yposition", "Zposition",
                                  "Xrotation", "Yrotation", "Zrotation"]:
                        raise SyntaxError("Syntax error in line %d: Invalid channel name: '%s'"%(self.linenr, tok)) #Si se encuentra un canal no valido, se lanza un error de sintaxis.                      
                    channels.append(tok) #Si es valido, se anade a la lista channels. 
                self._numchannels += len(channels) #Despues de leer todos los canales, se actualiza el numero total de canales self._numchannels
                self._nodestack[-1].channels = channels #se asigna la lista de canales al nodo actual en la pila _nodestack.
            elif tok=="JOINT": #C1
                node = Node()
                self._nodestack[-1].children.append(node) #Este nodo hijo se anade a la lista de hijos del nodo actual (children).
                self._nodestack.append(node) #Luego, se anade el nuevo nodo a la pila _nodestack
                self.readNode() #se llama recursivamente al metodo readNode para procesar este nuevo nodo.
            elif tok=="End": #C2 Si el token es "End", significa que se ha encontrado el final de una cadena de nodos, lo que indica un nodo sin hijos (final de una rama).
                node = Node()
                self._nodestack[-1].children.append(node)
                self._nodestack.append(node)
                self.readNode()
            elif tok=="}": #CERRAR Si el token es }, indica el final de la definicion del nodo actual.
                if self._nodestack[-1].isEndSite():
                    self._nodestack[-1].name = "End Site" #Si el nodo es un "End Site" (nodo final), se le asigna el nombre "End Site".
                self._nodestack.pop() #Luego el nodo actual se elimina de la pila _nodestack con pop() porque se ha terminado de procesar
                break #se rompe el bucle para terminar la funcion.
            else:
                raise SyntaxError("Syntax error in line %d: Unknown keyword '%s'"%(self.linenr, tok))
        

    # intToken
    def intToken(self):
        """Return the next token which must be an int.
        """

        tok = self.token()
        try:
            return int(tok)
        except ValueError:
            raise SyntaxError("Syntax error in line %d: Integer expected, got '%s' instead"%(self.linenr, tok))

    # floatToken
    def floatToken(self):
        """Return the next token which must be a float.
        """

        tok = self.token()
        try:
            return float(tok)
        except ValueError:
            raise SyntaxError("Syntax error in line %d: Float expected, got '%s' instead"%(self.linenr, tok))

    # token
    def token(self):
        """Return the next token."""

        # Are there still some tokens left? then just return the next one
        if self.tokenlist!=[]:
            tok = self.tokenlist[0]
            self.tokenlist = self.tokenlist[1:]
            return tok

        # Read a new line
        s = self.readLine()
        print("*********************************")
        print(s) #AQUI estoy imprimiendo todas las lineas que esta leyendo
        print("*********************************")
        print("")
        #Ejemplo s = "Hola Mundo Python"
        self.createTokens(s)
        #self.createTokens(s) convierte esa linea en la lista: ['Hola', 'Mundo', 'Python']
        #La primera vez que llames a token(), devolvera 'Hola', y la lista de tokens sera actualizada a:
        #['Mundo', 'Python']
        #La segunda vez que llames a token(), devolvera 'Mundo', y la lista de tokens sera:
        #['Python']
        return self.token()

    # readLine
    def readLine(self):
        """Return the next line.

        Empty lines are skipped. If the end of the file has been
        reached, a StopIteration exception is thrown.  The return
        value is the next line containing data (this will never be an
        empty string).
        """
        # Discard any remaining tokens
        # Descarta cualquier token que aun no haya procesado
        self.tokenlist = []
      
        # Read the next line
        # ANALISIS:
        #self.fhandle.readline() lee una linea del archivo asociado con self.fhandle y la devuelve como una cadena (string).
        #Cada vez que se llama a este metodo, se lee la siguiente linea disponible en el archivo, avanzando el "cursor" dentro del archivo.
        #Si el archivo llega al final (es decir, no hay mas lineas que leer), devuelve una cadena vacia ("").
        while 1:
            s = self.fhandle.readline()
            self.linenr += 1
            if s=="":
                raise StopIteration
            return s

    # createTokens
    def createTokens(self, s):
        """Populate the token list from the content of s.
        """
        #se encarga de convertir una linea de texto (almacenada en s) en tokens (partes individuales, como palabras o numeros) y almacenarlos 
        # en la lista de tokens del objeto (self.tokenlist).
        s = s.strip()
        #Descripcion: El metodo strip() elimina los espacios en blanco al principio y al final de una cadena de texto. Tambien elimina otros caracteres como saltos de linea o tabulaciones.
        #Devuelve: Una nueva cadena sin los espacios en blanco iniciales o finales.
        a = s.split()
        #Descripcion: El metodo split() divide una cadena de texto en partes (tokens), utilizando los espacios en blanco (por defecto) como delimitador. Tambien puedes especificar otro delimitador, como una coma o un punto.
        #Devuelve: Una lista de tokens (partes de la cadena).
        self.tokenlist = a

class BVHBroadcaster(BVHReader):
    def __init__(self, filename, root_frame):
        BVHReader.__init__(self, filename)
        self.br = tf.TransformBroadcaster()
        self.all_motions = []
        self.dt = 1
        self.num_motions = 1
        self.root_frame = root_frame

        self.counter = 0
        self.this_motion = None

        self.scaling_factor = 0.1

        # Posicion y rotacion para la transformacion de "map" a "world" usando tus valores ************************************************************************ 2 Kevin Inofuente Colque
        self.translation = (0.0, 0.0, 0.0)  # Valores x, y, z de la posicion ************************************************************************************** 2 Kevin Inofuente Colque
        self.rotation = tf.transformations.quaternion_from_euler(1.57, 0.0, 1.57)  # Valores yaw, pitch, roll convertidos a cuaterniones ************************** 2 Kevin Inofuente Colque


    #Mejoria del codigo para BVHs diferentes al del ejemplo - Kevin Inofuente Colque
    def onHierarchy(self, root):
        # Verifica si la componente en X es diferente de 0
        if root.children[0].children[0].offset[0] != 0:
            self.scaling_factor = 0.1 / root.children[0].children[0].offset[0]
            rospy.loginfo("Usando la componente X para el escalamiento.") #**************************************************************************************** 1 Kevin Inofuente Colque
        elif root.children[0].children[0].offset[1] != 0:  # Si X es 0, usa Y ************************************************************************************* 1 Kevin Inofuente Colque
            self.scaling_factor = 0.1 / root.children[0].children[0].offset[1] #*********************************************************************************** 1 Kevin Inofuente Colque
            rospy.loginfo("Usando la componente Y para el escalamiento.") #**************************************************************************************** 1 Kevin Inofuente Colque
        elif root.children[0].children[0].offset[2] != 0:  # Si X e Y son 0, usa Z ******************************************************************************** 1 Kevin Inofuente Colque
            self.scaling_factor = 0.1 / root.children[0].children[0].offset[2] #*********************************************************************************** 1 Kevin Inofuente Colque
            rospy.loginfo("Usando la componente Z para el escalamiento.") #**************************************************************************************** 1 Kevin Inofuente Colque
        else: #**************************************************************************************************************************************************** 1 Kevin Inofuente Colque
            raise ValueError("No se encontro un valor adecuado en X, Y o Z para el escalamiento.") #*************************************************************** 1 Kevin Inofuente Colque

    #def onHierarchy(self, root):
    #    self.scaling_factor = 0.1/root.children[0].children[0].offset[0]

    def onMotion(self, frames, dt):
        self.dt = dt
        self.num_motions = frames

    def onFrame(self, values):
        self.all_motions.append(values)

    def broadcastRootJoint(self, root, parent_frame):
        if root.isEndSite():
            return

        num_channels = len(root.channels)

        flag_trans = 0
        flag_rot = 0

        mat_rot = numpy.array([ [1.,0.,0.,0.], 
                                [0.,1.,0.,0.], 
                                [0.,0.,1.,0.], 
                                [0.,0.,0.,1.] ])

        for channel in root.channels:
            keyval = self.this_motion[self.counter] #Un array llamado self.this_motion en el indice self.counter.

            if(channel == "Xposition"): #Si el canal es "Xposition",
                flag_trans = True #marca la bandera de translacion (flag_trans) como verdadera
                x = keyval #y almacena el valor en x.
            elif(channel == "Yposition"): #Si el canal es "Yposition",
                flag_trans = True #marca la bandera de translacion (flag_trans) como verdadera
                y = keyval #y almacena el valor en y.
            elif(channel == "Zposition"): #Si el canal es "Zposition",
                flag_trans = True #marca la bandera de translacion (flag_trans) como verdadera
                z = keyval #y almacena el valor en z.

            elif(channel == "Xrotation"):
                flag_rot = True
                xrot = keyval
                theta = math.radians(xrot) #Si el canal es "Xrotation", primero convierte el valor de rotacion de grados a radianes
                c = math.cos(theta) #luego calcula el coseno y el seno del angulo de rotacion.
                s = math.sin(theta)
                mat_x_rot = numpy.array([ [1.,0.,0.,0.], 
                                          [0., c,-s,0.], 
                                          [0., s, c,0.], 
                                          [0.,0.,0.,1.] ])
                mat_rot = numpy.matmul(mat_rot, mat_x_rot)

            elif(channel == "Yrotation"):
                flag_rot = True
                yrot = keyval
                theta = math.radians(yrot)
                c = math.cos(theta)
                s = math.sin(theta)
                mat_y_rot = numpy.array([ [ c,0., s,0.],
                                          [0.,1.,0.,0.],
                                          [-s,0., c,0.],
                                          [0.,0.,0.,1.] ])
                mat_rot = numpy.matmul(mat_rot, mat_y_rot)

            elif(channel == "Zrotation"):
                flag_rot = True
                zrot = keyval
                theta = math.radians(zrot)
                c = math.cos(theta)
                s = math.sin(theta)
                mat_z_rot = numpy.array([ [ c,-s,0.,0.],
                                          [ s, c,0.,0.],
                                          [0.,0.,1.,0.],
                                          [0.,0.,0.,1.] ])
                mat_rot = numpy.matmul(mat_rot, mat_z_rot)
            else:
                return
            self.counter += 1
        
        if flag_trans:
            temp_trans = (self.scaling_factor * (x + root.offset[0]), 
                          self.scaling_factor * (y + root.offset[1]), 
                          self.scaling_factor * (z + root.offset[2]))
        else:
            temp_trans = (self.scaling_factor * (root.offset[0]), 
                          self.scaling_factor * (root.offset[1]), 
                          self.scaling_factor * (root.offset[2]))

        temp_rot = tf.transformations.quaternion_from_matrix(mat_rot)

        self.br.sendTransform(temp_trans, temp_rot, rospy.Time.now(), root.name, parent_frame)

        # Publicar la transformacion de `map` a `world` usando la posicion y rotacion especificadas *************************************************************** 2 Kevin Inofuente Colque
        self.br.sendTransform(self.translation, self.rotation, rospy.Time.now(), "world", "map") # Frame hijo es World y Frame padre es Map *********************** 2 Kevin Inofuente Colque

        for each_child in root.children:
            self.broadcastRootJoint(each_child, root.name)

    def broadcast(self, loop=False): # 1 PRIMERA FUNCION QUE LLAMA -----------------------------------------------------------------------------
        self.read() # 2 SEGUNDA FUNCION QUE LLAMA ----------------------------------------------------------------------------------------------
        
        #Estamos definiendo la frecuencia en 70 porque esa es la frecuencia en que PEPPER publica los TF
        rate = rospy.Rate(70) #(1/self.dt) ************************************************************************************************************************ 3 Kevin Inofuente Colque

        frame_interval = 10

        while not rospy.is_shutdown():
            for ind in range(self.num_motions):#, frame_interval):
                self.counter = 0
                self.this_motion = self.all_motions[ind]

                #Publicar las articulaciones del esqueleto
                self.broadcastRootJoint(self._root, self.root_frame)

                if rospy.is_shutdown():
                    break

                rate.sleep()

            if not loop:
                break

def argsparser():
    parser = argparse.ArgumentParser("python BVHBroadcaster.py")
    parser.add_argument('bvh_file', help="A path to bvh file that you want to broadcast")
    parser.add_argument('base_frame', help="An existing frame in rviz on which the skeleton will be loaded")
    parser.add_argument('-n', '--name', help="Node name, default: BVHBroadcaster", default="BVHBroadcaster")
    parser.add_argument('-l', '--loop', help="Loop broadcasting", action="store_true")
    return parser.parse_args()

def main(args):
    rospy.init_node(args.name)

    # file_name = "/home/mingfei/Documents/RobotManipulationProject/mocap/62/62_07.bvh"
    bvh_test = BVHBroadcaster(args.bvh_file, args.base_frame)
    # bvh_test hereda todos los atributos y funciones de la clase BVHBroadcaster presente en este modulo de python **********************************
    rospy.loginfo("Broadcasting bvh file (%s) on frame %s"%(args.bvh_file, args.base_frame))

    if args.loop:
        rospy.loginfo("Loop")
    else: 
        rospy.loginfo("Only once")
    bvh_test.broadcast(loop=args.loop) # 1 PRIMERA FUNCION QUE LLAMA -----------------------------------------------------------------------------
    rospy.loginfo("Broadcasting done")

def test():
    rospy.init_node("BVHBroadcaster")
    file_name = "/home/mingfei/Documents/projects/RobotManipulationProject/mocap/62/62_07.bvh"
    bvh_test = BVHBroadcaster(file_name, "world")
    bvh_test.broadcast(loop=True)


if __name__ == "__main__":
    args = argsparser()
    main(args)

#rosrun tf static_transform_publisher 0.0.0 0.0 1.57 0.0 1.57 map world 10