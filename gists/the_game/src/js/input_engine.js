const InputEngine = (in_canvas) => {
  const
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
  is_shift_down = () => key.shift,
  is_ctrl_down = () => key.ctrl,
  is_alt_down = () => key.alt,
  get_mouse = () => ou.copy(mouse),
  get_code = () => key.code

  canvas.onmousemove = ({ offsetX: x, offsetY: y, which }) => {
    ou.assign(mouse, { x, y, which })
  }
  
  canvas.onmousedown = ({ offsetX: x, offsetY: y, which }) => {
    ou.assign(mouse, { x, y, which, down: true })
  }

  canvas.onmouseup = ({ offsetX: x, offsetY: y, which }) => {
    ou.assign(mouse, { x, y, which, down: false })
  }
  
  document.onkeydown = ({ keyCode }) => {
    key.code = keyCode
    switch (key.code) {
      case 17:
        key.ctrl = true
        break
      case 18:
        key.alt = true
        break
    }
  }

  document.onkeyup = ({ keyCode }) => {
    key.code = keyCode
    switch (key.code) {
      case 17:
        key.ctrl = false
        break
      case 18:
        key.alt = false
        break
    }
  }
  
  return ({
    is_shift_down,
    is_ctrl_down,
    is_alt_down,
    get_mouse,
    get_code,
  })
}