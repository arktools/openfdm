#!/usr/bin/python

datcom = open('datcom_aero.xml','r')
lines = datcom.readlines()
datcom.close()

data = ['#1\n']
for n in range(len(lines)):
	if lines[n].find('<function name')!=-1:
	    start = lines[n].rfind('/')
	    end = lines[n].rfind('"')
	    name = lines[n][start+1:end]
	    data.append('# '+lines[n])        
	    a = n+1
	    while a<len(lines)-1 and lines[a].find('<tableData>')==-1 and lines[a].find('</function>')==-1:
		data.append('# '+lines[a])
		if lines[a].find('<value>')!=-1:
		    index = a
		a = a+1
	    if lines[a].find('<tableData>')==-1:
		data.insert(index,'double '+str(name)+'(2,2)\n')
		vstart = lines[index].find('>')+1
		vend = lines[index].rfind('<')
		value = lines[index][vstart:vend]
		data.insert(index+1,'0.0\t'+value+'\n')
		data.insert(index+2,'1.0\t'+value+'\n')
		            
	elif lines[n].find('<tableData>')!=-1:
	    data.append('# '+lines[n])
	    data.append('\n')
	    tablestart = len(data)-1
	    a = n+1
	    while a>n and a<len(lines)-1 and lines[a].find('</tableData>')==-1:
		data.append(lines[a])
		a = a+1
	    rows = a-n-1
	    if len(lines[n+1].split())<=2 or len(lines[n+1].split())>rows:
		data[tablestart]='double '+name+'('+str(rows)+','+str(len(lines[n+1].split()))+')\n'
	    else:
		  data[tablestart]='double '+name+'('+str(len(lines[n+1].split())+1)+','+str(rows)+')\n'            
		
	elif lines[n].find('</tableData>')!=-1:
	    data.append('# '+lines[n])
	    a = n+1
	    while a>n and a<len(lines)-1 and lines[a].find('</function>')==-1 and lines[a].find('<tableData>')==-1:
		data.append('# '+lines[a])
		a = a+1
	    if lines[a].find('<tableData>')!=-1:
		name = name+'_2'

	elif lines[n].find('</function>')!=-1:
	    data.append('# '+lines[n])
	    a = n+1
	    while a>n and a<len(lines)-1 and lines[a].find('<function name')==-1:
		data.append('# '+lines[a])
		a = a+1
    
arr = []
for line in range(len(data)):
	if line <len(data)-1 and data[line].find('double') != -1 and len(data[line+1].split())>2:
	    start = line+1
	    if float(data[line+1].split()[0]) > float(data[line+2].split()[0]):
		arr.append([str(float(data[line+2].split()[0])-1)]+data[line+1].split())
	    else:
		arr.append([str(float(data[line+1].split()[0])-1)]+data[line+1].split())              
	    for n in range(line+2,len(data)-1):
		if data[n].find('#')==-1:
		    arr.append(data[n].split())
		else:
		    end = n-2
		    num = start
		    for line in zip(*arr):
		        data[num]=''
		        for n in range(len(line)):
		            data[num] = data[num]+line[n]+'\t'
		        data[num]=data[num]+'\n'
		        num = num+1
		    for n in range(num,end+2):
		        if data[n].find('</tableData>')==-1:
		            data[n]=''
		        else:
		            break
		    arr = []
		    break

modelica = open('modelica.txt','w')
for line in data:
	modelica.write(line)

modelica.close()
