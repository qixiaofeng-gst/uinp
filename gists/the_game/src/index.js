/*PUT js/shader_creator.js */

const prgm_drop = create_shader_program('vs_share', 'fs_drop')
const prgm_update = create_shader_program('vs_share', 'fs_update')
const prgm_render = create_shader_program('vs_render', 'fs_render')
const image = document.getElementById('texture')
const resolution = 128
const perturbance = 0.03
const drop_radius = 10 //px
const strength = 120 // 0.04 - 0.08
const texture_delta = new Float32Array([1 / resolution, 1 / resolution])
const top_left = new Float32Array([0, 0])
const bottom_right = new Float32Array([1, 1])
const container_ratio = new Float32Array([1, 0.68])
const tex_background = gl.createTexture()
const tex_bff_arr = []
const tex_bff_index = {
  write: 0,
  read: 1,
  swap: () => {
    tex_bff_index.write = 1 - tex_bff_index.write
    tex_bff_index.read = 1 - tex_bff_index.read
    //console.log(tex_bff_index)
  }
}
const bff_quad = gl.createBuffer()
const arr_vtx_quad = [
  -1, -1,
  +1, -1,
  +1, +1,
  -1, +1,
]
const draw_quad = () => {
  gl.bindBuffer(gl.ARRAY_BUFFER, bff_quad)
  gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0)
  gl.drawArrays(gl.TRIANGLE_FAN, 0, 4)
  //gl.drawArrays(gl.LINE_LOOP, 0, 4)
}
const init_scene = () => {
  prgm_drop.use()
  gl.enableVertexAttribArray(0)
  prgm_update.use()
  gl.enableVertexAttribArray(0)
  prgm_update.delta = texture_delta
  prgm_render.use()
  gl.enableVertexAttribArray(0)
  prgm_render.delta = texture_delta

  gl.bindBuffer(gl.ARRAY_BUFFER, bff_quad)
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(arr_vtx_quad), gl.STATIC_DRAW)

  gl.bindTexture(gl.TEXTURE_2D, tex_background)
  gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, 1)
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR)
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR)
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE) // gl.REPEAT perhaps
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE) // gl.REPEAT perhaps
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, image)

  for (let i = 0; i < 2; ++i) {
		const framebuffer = gl.createFramebuffer()
    const texture = gl.createTexture()
    const arr_framebuffer = new Float32Array(resolution * resolution * 4)

		gl.bindFramebuffer(gl.FRAMEBUFFER, framebuffer)
		gl.bindTexture(gl.TEXTURE_2D, texture)

		gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR)
		gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR)
		gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.REPEAT)
		gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.REPEAT)
		gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, resolution, resolution, 0, gl.RGBA, gl.FLOAT, arr_framebuffer)

		gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, texture, 0)
    tex_bff_arr.push({
      texture,
      framebuffer
    })
  }

  gl.clearColor(0, 0, 0, 0)
  gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)
}
init_scene()
gl.canvas.onclick = ({ offsetX, offsetY }) => {
  console.log(offsetX, offsetY)
  drop(offsetX, offsetY, drop_radius, strength)
  //update()
}
const drop = (x, y, radius, strength) => {
  const width = gl.canvas.width
  const height = gl.canvas.height
  const drop_position = new Float32Array([
    (2 * x - width) / width,
    (height - 2 * y) / width,
  ])

  prgm_drop.use()
  gl.viewport(0, 0, resolution, resolution)
  gl.bindFramebuffer(gl.FRAMEBUFFER, tex_bff_arr[tex_bff_index.write].framebuffer)
  prgm_drop.texture = tex_bff_arr[tex_bff_index.read].texture

  prgm_drop.center = drop_position
  prgm_drop.radius = radius
  prgm_drop.strength = strength

  draw_quad()
  tex_bff_index.swap()
}
const update = () => {
  prgm_update.use()
  gl.viewport(0, 0, resolution, resolution)
  gl.bindFramebuffer(gl.FRAMEBUFFER, tex_bff_arr[tex_bff_index.write].framebuffer)
  prgm_update.texture = tex_bff_arr[tex_bff_index.read].texture
  draw_quad()
  tex_bff_index.swap()
}
const render = () => {
  prgm_render.use()
  gl.viewport(0, 0, gl.canvas.width, gl.canvas.height)
  gl.enable(gl.BLEND)
  gl.bindFramebuffer(gl.FRAMEBUFFER, null)
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

  prgm_render.perturbance = perturbance
  prgm_render.top_left = top_left
  prgm_render.bottom_right = bottom_right
  prgm_render.container_ratio = container_ratio

  prgm_render.sampler_bg = tex_background
  prgm_render.sampler_ripples = tex_bff_arr[0].texture

  draw_quad()
  gl.disable(gl.BLEND)
}
const main_loop = () => {
  update()
  render()
}
const looper = cb => {
  cb()
  requestAnimationFrame(delta_time => {
    looper(cb)
  })
}
looper(main_loop)
