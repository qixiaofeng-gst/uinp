const ou = require('./object_utilities.js')

const InputEngine = (in_canvas) => {
  const
  shift = 16,
  ctrl = 17,
  alt = 18,
  a = 65,
  alphabet = 'abcdefghijklmnopqrstuvwxyz',
  zero = 48,
  numbers = '0123456789',
  left = 37,
  up = 38,
  right = 39,
  down = 40,
  enter = 13,
  f1 = 112,
  functions = [
    'f1', 'f2', 'f3', 'f4', 'f5', 'f6',
    'f7', 'f8', 'f9', 'f10', 'f11', 'f12',
  ],
  insert = 45,
  del = 46,
  back = 8,
  curve = 192,
  esc = 27,
  minus = 189,
  plus = 187,
  cmd = 91,
  
  key = {
    shift: false,
    ctrl: false,
    alt: false,
    code: 0,
  },
  mouse = {
    x: 0,
    y: 0,
    which: 3,
    down: false,
  },
  listeners = {
    onmousemove: [],
    onmousedown: [],
    onmouseup: [],
    onclick: [],
    
    onkeypress: [],
    onkeydown: [],
    onkeyup: [],
  },
  
  is_shift_down = () => key.shift,
  is_ctrl_down = () => key.ctrl,
  is_alt_down = () => key.alt,
  get_mouse = () => ou.copy(mouse),
  get_code = () => key.code

  canvas.onmousemove = ({ offsetX: x, offsetY: y, which }) => {
    ou.assign(mouse, { x, y, which })
    
    for (const o of listeners.onmousemove) {
      o()
    }
  }
  
  canvas.onmousedown = ({ offsetX: x, offsetY: y, which }) => {
    ou.assign(mouse, { x, y, which, down: true })
    
    for (const o of listeners.onmousedown) {
      o()
    }
  }

  canvas.onmouseup = ({ offsetX: x, offsetY: y, which }) => {
    ou.assign(mouse, { x, y, which, down: false })
    
    for (const o of listeners.onmouseup) {
      o()
    }
  }
  
  canvas.onclick =  ({ offsetX: x, offsetY: y, which }) => {
    ou.assign(mouse, { x, y, which })
    
    for (const o of listeners.onclick) {
      o()
    }
  }
  
  document.onkeypress = ({ keyCode }) => {
    key.code = keyCode
    for (const o of listeners.onkeypress) {
      o()
    }
  }
  
  document.onkeydown = ({ keyCode }) => {
    key.code = keyCode
    switch (key.code) {
      case shift:
        if (key.shift) {
          return
        }
        key.shift = true
        break
      case ctrl:
        if (key.ctrl) {
          return
        }
        key.ctrl = true
        break
      case alt:
        if (key.alt) {
          return
        }
        key.alt = true
        break
    }
    
    for (const o of listeners.onkeydown) {
      o()
    }
  }

  document.onkeyup = ({ keyCode }) => {
    key.code = keyCode
    switch (key.code) {
      case shift:
        key.shift = false
        break
      case ctrl:
        key.ctrl = false
        break
      case alt:
        key.alt = false
        break
    }
    
    for (const o of listeners.onkeyup) {
      o()
    }
  }
  
  return new Proxy({
    is_shift_down,
    is_ctrl_down,
    is_alt_down,
    get_mouse,
    get_code,
    
    shift,
    ctrl,
    alt,
  }, {
    set: (_, key, val, __) => {
      const
      off = 'off',
      offall = `${off}all`,
      msg = `No property [${key}] in the Input Engine`
      
      if (key === offall && true === val) {
        for (const name in listeners) {
          const e = listeners[name]
          e.splice(0, e.length)
        }
        return val
      }
      if (key.startsWith(off)) {
        const mapped = key.replace(off, 'on')
        if (listeners.hasOwnProperty(mapped)) {
          const container = listeners[mapped]
          if (container.includes(val)) {
            container.splice(container.indexOf(val), 1)
          }
        } else {
          console.warn(msg)
        }
      } else {
        if (listeners.hasOwnProperty(key)) {
          const container = listeners[key]
          if (false == container.includes(val)) {
            container.push(val)
          }
        } else {
          console.warn(msg)
        }
      }
    }
  })
}

module.exports = InputEngine