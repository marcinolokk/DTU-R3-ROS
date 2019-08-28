msg = "Hello world"
print(msg)


import urllib.request
import webbrowser
i_odl = 22960
i = 22901
while i<=23000:
    url = "https://dtu.onlineeksamen.dk/assignment/assignmentfile/%s" % (i)
    webbrowser.open_new_tab(url)
    i +=1

