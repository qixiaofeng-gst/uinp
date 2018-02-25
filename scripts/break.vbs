'set wsh = createobject("wscript.Shell")

'do
'wscript.sleep 1000
'wsh.sendKeys "^ "
'loop

do
sleeptime = cint(inputbox("How many minutes:", "Minutes Input", 0))

if sleeptime = 0 then
exit do
end if

st = sleeptime * 60000

wscript.sleep st

set obj = createobject("shell.application")
obj.MinimizeAll

'msgbox "Take a break." & sleeptime, 1, "^_^" & st
msgbox "Take a break.", 1, "^_^"
loop