const
{
  start_player,
} = require('./player.js'),
active_clz = 'active',
setup = (be, ie) => {
  return ({
    basic: btn => {
      console.log('Editor: basic')
      ie.onkeyup = () => {
        if (ie.ctrl == ie.get_code()) {
        }
      }
      
      ie.onkeydown = () => {
        if (ie.ctrl == ie.get_code()) {
        }
      }
    },
    mark_fix: btn => {
      console.log('Editor: mark fix')
    },
    move_fix: btn => {
      console.log('Editor: move fix')
    },
    mark_edge: btn => {
      console.log('Editor: mark edge')
    },
    make_line: btn => {
      console.log('Editor: make line')
    },
    make_square: btn => {
      console.log('Editor: make square')
    },
    make_cycle: btn => {
      console.log('Editor: make cycle')
    },
    make_cloth: btn => {
      console.log('Editor: make cloth')
    },
    make_rope: btn => {
      console.log('Editor: make rope')
    },
    make_stave: btn => {
      console.log('Editor: make stave')
    },
    make_breakable: btn => {
      console.log('Editor: make breakable')
    },
    exit_editor: btn => {
      console.log('Exit editor, player take over events')
      start_player(be, ie)
      remove_class(btn, active_clz)
    },
  })
},
add_class = (ele, clz) => {
  ele.className += ` ${clz}`
},
remove_class = (ele, clz) => {
  const old = ele.className
  ele.className = old.replace(clz, '').trim()
},
create_editor = (be, ie) => {
  /**
  be: bone engine
  ie: input engine
  */
  let
  state = false
  
  const
  config = setup(be, ie)
  
  for (const entry in config) {
    const
    selector = `#e_${entry}`,
    button = document.querySelector(selector)
    
    if (undefined == button) {
      continue
    }
    
    button.onclick = () => {
      const
      old = document.querySelector(`li.${active_clz}`)
      
      if (old) {
        remove_class(old, active_clz)
      }
      add_class(button, active_clz)
      
      ie.offall = true
      be.set_kinematics(false)
      config[entry](button)
    }
  }
}

module.exports = {
  create_editor,
}