#!/usr/bin/env pypy
""" 
--------------------------------------------------------
  PROGRAMA DE PRUEBA: DEBUGEADO DE REGLAS JERARQUICAS
--------------------------------------------------------
 Cuando la primera regla de un plan es una regla jerarquica, esta se descompone
 en sus subreglas hasta llegar a una regla normal (atomica) que el robot pueda
 ejecutar. Este programa debugea ese tipo de reglas.
"""
import sys, traceback

sys.path.append('/usr/local/share/agm/')

import xmlModelParser
from AGGL import *

from parseAGGL import AGMFileDataParsing
from agglplanner import *

def quitar_Constantes_Creadas(ficheroMundo):
	constantes = []
	constante = []
	"""Abrimos el fichero del mundo origen y sacamos todos los numeros enteros (variables constantes) del fichero.  Para ello solo nos quedaremos con
	las declaraciones del tipo: <symbol id="1" type="object">"""
	init = open(ficheroMundo)
	linea = init.readline()
	while linea !="":
		if linea.find('<symbol')!=-1:
			"""Sacamos la parte id="1" que define el nodo como constante o variable. Minimo, esta parte mide 6
			de longitud. El identificador numerico empieza en la posicion 4.
			Su longitud depende del numero que tenga dentro de las comillas"""
			vector = linea.split()
			i = 4
			while i<int(len(vector[1])-1):
				constante.append(vector[1][i])
				i=i+1
			"""Pasamos a entero la cadena con el id del nodo, lo guardamos en el vector de constantes
			y limpiamos las variables usadas."""
			T2 = int(''.join(constante)) 
			constantes.append(T2)
			constante = []				
		linea = init.readline()
	init.close()
	
	eliminar = []
	correspondencia = False
	contenido=""
	"""Abrimos el fichero target y vamos comparando todas las lineas  <symbol id="1" type="object">
	con el vector de constantes que hemos sacado antes. Si hay una constante que no esta recogida en el
	vector de constantes es que es nueva y debemos cambiar su nombre para hacerla variable."""
	target = open("/tmp/resultado.xml")
	linea = target.readline()
	while linea !="":
		if linea.find('<symbol')!=-1:
			print linea
			"""Si hemos encontrado una linea que define un nuevo nodo, comparamos el identificador
			del nodo con todas las constantes originales """
			for const in constantes:
				if linea.find('id="'+const.__str__()+'"')!=-1:
					correspondencia = True
			"""Si no hemos encontrado correspondencia ninguna, es que es un nuevo nodo creado a partir
			de la ejecucion de la regla jerarquica. Guardamos su identificador para despues cambiarlo por
			otro valor (lo pasaremos de constante a variable)"""
			if correspondencia==False:
				print 'NO ENCONTRADO'
				vector = linea.split()
				i = 4
				while i<int(len(vector[1])-1):
					constante.append(vector[1][i])
					i=i+1
				T2 = int(''.join(constante)) 
				eliminar.append(T2)
				constante = []		
		contenido = contenido+linea
		linea = target.readline()
	target.close()

	if len(eliminar)>0:
		"""Si hay elementos a eliminar/cambiar, buscamos sus apariciones y las modificamos"""
		print 'hay chicha'
		cadenaLimpia=[contenido]
		import re
		target = open("/tmp/resultado.xml",'w')    #abrimos el fichero con permisos de escritura
		i=0
		for el in eliminar:
			print el
			patter = re.compile('"'+el.__str__()+'"', re.I | re.S)
			cadenaLimpia.append(patter.sub('"AA'+el.__str__()+'"', cadenaLimpia[i]))
			i = i+1
		print cadenaLimpia
		target.write(cadenaLimpia[len(cadenaLimpia)-1])    #escribimos la cadena ya actualizada y sin la regla jerarquica
		target.close() 




def eliminar_Regla(ficheroDominio, nombreRegla):
	f = open(ficheroDominio)
	#cadena = f.read()   
	#f.close()
	"""Como podemos eliminar una regla del fichero. Veamos su estructura:
	hierarchical nombre de la regla : active (1)
	{
		{
			PRECONDICIONES
		}
		{
			POSTCONDICIONES
		}
		effect
		{
			OPCIONAL
		}
	}
	Podemos recorrer el fichero desde la regla que queremos hasta que aparezca
	la siguiente regla o finalice el fichero"""
	hierarchical = 0
	encontrado = False
	linea = f.readline()
	while linea !="":
		if linea=='hierarchical '+nombreRegla+' : active(1)\n':
			encontrado = True
		elif linea.find('hierarchical')!=-1 and encontrado==True and hierarchical==0:
			hierarchical = hierarchical+1;
			break
		linea = f.readline()

	f.close()
	
	f = open(ficheroDominio)
	cadena = f.read()
	f.close()
	
	if hierarchical>0: 
		"""Si hierarchical es mayor que 0 es que la regla que queremos eliminar
		no es la ultima, entonces buscamos y reemplazamos"""
		import re
		patter = re.compile('hierarchical '+nombreRegla+' : .*?hierarchical ', re.I | re.S)
		cadenaLimpia = patter.sub("hierarchical ", cadena)
	else:
		inicio = cadena.find('hierarchical '+nombreRegla) #inicio de la regla	
		cadenaLimpia=cadena[0:inicio]
		
	otro = open("/tmp/copiaDominio.aggl",'w')    #abrimos el fichero con permisos de escritura
	otro.write(cadenaLimpia)    #escribimos la cadena ya actualizada y sin la regla jerarquica
	otro.close()    #cerramos el fichero"""
	




if __name__ == '__main__': 
	"""
	 COSAS QUE DEBEMOS PASARLE AL PROGRAMA:
		1) El origen del problema (el init.xml)
		2) La gramatica donde se encuentra la regla jerarquica
		3) El nombre de la regla (o el plan donde sacar el nombre de la regla)
		4) Los parametros de la regla. Si damos el plan
	"""
	if len(sys.argv)<3:
		print 'Usage\n\t', sys.argv[0], ' domain.aggl init.xml plan.p'
	else:
		"""
		Si lo tenemos todo ya preparado, vamos sacando los ficheros y datos que vamos a usar
		"""
		ficheroDominio = sys.argv[1] # Fichero AGGL que recoge las reglas del dominio.
		ficheroMundo   = sys.argv[2] # Fichero XML con el mundo inicial
		ficheroPlan    = sys.argv[3] # Fichero P con el plan para alcanzar el mundo target.
		
		"""
		Sacamos el nombre de la regla jerarquica
		nombreRegla @  {   x:y x:y ...}      \n
		            n n+1  n+2        len-2  len-1
		"""
		f = open(ficheroPlan, "r")
		reglaJerarquica = f.readline()
		f.close()

		finNombre = reglaJerarquica.find("@")
		nombreRegla = reglaJerarquica[:finNombre]
		
		""" Aplicamos la regla sobre el mundo de origen y guardamos el resultado en un fichero temporal. Para ello, debemos... """
		"1) pasar el dominio de AGGL a PY" 
		agmData = AGMFileDataParsing.fromFile(ficheroDominio)
		agmData.generateAGGLPlannerCode("/tmp/domain.py", skipPassiveRules=True)
		dominioPython = "/tmp/domain.py"
		
		"2) Sacamos del dominio el conjunto de reglas"
		dominioFinal = imp.load_source('domain', dominioPython).RuleSet() 
		mapaReglas = copy.deepcopy(dominioFinal.getRules())
		
		"3) Preparamos el grafo de inicio"
		mundoInicio = WorldStateHistory(xmlModelParser.graphFromXML(ficheroMundo))
		mundoInicio.nodeId = 0 
		
		"4) Del mapa de reglas buscamos nuestra regla jerarquica y la aplicamos. Guardamos el resultado en un XML"
		for resultado in mapaReglas[nombreRegla](mundoInicio): print ' '
		resultado.graph.toXML("/tmp/resultado.xml")
		"5) Del fichero resultado tenemos que poner los nuevos nodos creados como variables no como constantes, para que el programa funcione: "
		quitar_Constantes_Creadas(ficheroMundo)
		
		"AHORA, una vez que tenemos el resultado de aplicar esa regla, la quitamos del dominio de reglas"	
		eliminar_Regla(ficheroDominio, nombreRegla)
		"Guardamos la ruta del fichero de dominio modificado sin la regla"
		ficheroDominioSinRegla = "/tmp/copiaDominio.aggl"
		
		"Llamamos al plan para que mire si el target guardado en result.xml es alcanzable sin la regla jerarquica"
		import subprocess
		subprocess.call(["agglplan", ficheroDominioSinRegla, ficheroMundo, "/tmp/resultado.xml"])
