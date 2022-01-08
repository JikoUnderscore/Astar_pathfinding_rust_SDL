#![allow(non_snake_case)]

use std::collections::{VecDeque};
use sdl2::render::WindowCanvas;
use sdl2::{Sdl, TimerSubsystem};
use sdl2::event::Event;
use sdl2::keyboard::Scancode;
use sdl2::mouse::MouseButton;
use sdl2::rect::{Point, Rect};

pub struct Renderer {
    pub sdl_context: Sdl,
    pub ekran: WindowCanvas,
    pub fps_ctrl: FpsCapDeltaTime,
}

impl Renderer {
    pub fn new(title: &str) -> Renderer {

        // init systems.
        let sdl_context = sdl2::init().unwrap();
        let video_subsystem = sdl_context.video().unwrap();

        // create a window.
        let mut win = video_subsystem.window(title, 1280, 720);
        let window = win.position_centered()
                        .resizable()
                        .build()
                        .unwrap();


        // get the canvas
        let ekran = window.into_canvas().build().unwrap();
        // ekran.set_logical_size(1280, 720).unwrap();


        Renderer {
            ekran,
            fps_ctrl: FpsCapDeltaTime::new(sdl_context.timer().unwrap(), 60),
            sdl_context,
        }
    }
}

pub struct FpsCapDeltaTime {
    pub ttime: TimerSubsystem,
    frame_delay: u32,
    pub fps: f64,

    cap_frame_start: u32,
    cap_frame_end: u32,

    pub dt: f64,
    fps_last_time: u64,
    fps_now: u64,
}

impl FpsCapDeltaTime {
    pub fn new(ttime: TimerSubsystem, fps: u32) -> Self {
        Self {
            ttime,
            frame_delay: 1000 / fps,
            fps: fps as f64,
            cap_frame_start: 0,
            cap_frame_end: 0,
            dt: 0.0,
            fps_last_time: 0,
            fps_now: 0
        }
    }

    pub fn start(&mut self) {
        self.cap_frame_start = self.ttime.ticks();

        self.fps_last_time = self.fps_now;
        self.fps_now = self.ttime.performance_counter();

        self.dt = ((self.fps_now - self.fps_last_time) * 1000) as f64 / self.ttime.performance_frequency() as f64;
    }

    pub fn end(&mut self) {
        self.cap_frame_end = self.ttime.ticks() - self.cap_frame_start;
        if self.cap_frame_end < self.frame_delay {
            self.ttime.delay(self.frame_delay - self.cap_frame_end);
        }
    }
}

struct SNode<'node> {
    obstacle: bool,
    visited: bool,
    global_goal: f64,
    local_goal: f64,
    point: Point,
    neighbours: Vec<&'node mut SNode<'node>>,
    parent: Option<*const SNode<'node>>
}


impl<'node> SNode<'node> {
    fn distance(&self, other: &SNode) -> f64 {
        // (a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y)

        let a = (self.point.x() - other.point.x()) * (self.point.x() - other.point.x());
        let b = (self.point.y() - other.point.y()) * (self.point.y() - other.point.y());
        return ((a + b) as f64).sqrt()
    }


    pub fn heuristic(&self, other: *const SNode) -> f64 {
        unsafe { return self.distance(&*other); }
    }
}

fn solve_astar<'a>(node_start_mut_ptr: *mut SNode<'a>, node_end: *const SNode<'a>) {
    let mut curen_node = node_start_mut_ptr;
    unsafe {
        (*curen_node).local_goal = 0.0;
        (*curen_node).global_goal = (*curen_node).heuristic(node_end);
    }

    let mut list_not_tested_nodes = VecDeque::new();
    list_not_tested_nodes.push_back(curen_node);

    while !list_not_tested_nodes.is_empty() && (node_end != curen_node) {
        unsafe {
            list_not_tested_nodes.make_contiguous().sort_by(|lhs, rhs| { (*(*lhs)).global_goal.partial_cmp(&(*(*rhs)).global_goal).unwrap() });

            while !list_not_tested_nodes.is_empty() && (*(*list_not_tested_nodes.front().unwrap())).visited {
                list_not_tested_nodes.pop_front();
            }
        }

        if list_not_tested_nodes.is_empty() {
            break;
        }

        unsafe {
            curen_node = *list_not_tested_nodes.front_mut().unwrap();
            (*curen_node).visited = true;


            for neighbour in (*curen_node).neighbours.iter_mut() {
                if !neighbour.visited && !neighbour.obstacle {
                    list_not_tested_nodes.push_back(*neighbour);
                }

                let possibly_lower_goal = (*curen_node).local_goal + (*curen_node).distance(*neighbour);
                if possibly_lower_goal < neighbour.local_goal {
                    neighbour.parent = Some(curen_node);
                    neighbour.local_goal = possibly_lower_goal;
                    neighbour.global_goal = neighbour.local_goal + neighbour.heuristic(node_end);
                }
            }
        }
    }
}


/// https://www.youtube.com/watch?v=icZj67PTFhc
fn main() {
    const MAP_WIDHT: usize = 15;
    const MAP_HEIGHT: usize = 50;
    const START_OFFSET: i32 = 0;

    let mut core = Renderer::new("a start pathing");


    let mut event_pump = core.sdl_context.event_pump().unwrap();


    let mut list_rects = vec![];
    let mut nodes_list = vec![];


    for x in START_OFFSET..MAP_WIDHT as i32 + START_OFFSET {
        for y in START_OFFSET..MAP_HEIGHT as i32 + START_OFFSET {
            let pos_rect = Rect::new(x * 20, y * 20, 10, 10);
            list_rects.push(pos_rect);
            nodes_list.push(SNode {
                obstacle: false,
                visited: false,
                global_goal: f64::INFINITY,
                local_goal: f64::INFINITY,
                point: pos_rect.center(),
                neighbours: vec![],
                parent: None
            })
        }
    }
    {
        let node_list_ptr: *mut _ = &mut nodes_list;

        for x in 0..MAP_WIDHT.max(MAP_HEIGHT) {
            for y in 0..MAP_HEIGHT.max(MAP_WIDHT) {
                unsafe {
                    if y > 0 && y * MAP_HEIGHT + x < nodes_list.len() {
                        nodes_list[y * MAP_HEIGHT + x].neighbours.push(&mut (*node_list_ptr)[(y - 1) * MAP_HEIGHT + (x + 0)]);
                    }

                    if y < MAP_HEIGHT.max(MAP_WIDHT) - 1 && (y + 1) * MAP_HEIGHT + (x + 0) < nodes_list.len() {
                        nodes_list[y * MAP_HEIGHT + x].neighbours.push(&mut (*node_list_ptr)[(y + 1) * MAP_HEIGHT + (x + 0)]);
                    }

                    if x > 0 && x < MAP_HEIGHT && y * MAP_HEIGHT + x < nodes_list.len() {
                        nodes_list[y * MAP_HEIGHT + x].neighbours.push(&mut (*node_list_ptr)[(y + 0) * MAP_HEIGHT + (x - 1)]);
                    }

                    let map_w_h = if MAP_WIDHT > MAP_HEIGHT { (MAP_WIDHT, MAP_HEIGHT) } else { (MAP_HEIGHT, MAP_HEIGHT) };
                    if x < map_w_h.0 - 1 && x < map_w_h.1 - 1 && (y + 0) * MAP_HEIGHT + (x + 1) < nodes_list.len() {
                        nodes_list[y * MAP_HEIGHT + x].neighbours.push(&mut (*node_list_ptr)[(y + 0) * MAP_HEIGHT + (x + 1)]);
                    }
                }
            }
        }
    }

    let mut node_start_mut_ptr: *mut _ = &mut nodes_list[0];
    let mut node_end_ptr: *const _ = &nodes_list[nodes_list.len() - 1];

    let mut is_running = true;

//--------- LOOP
    while is_running {
        core.fps_ctrl.start();
        core.ekran.set_draw_color((0, 0, 0));
        core.ekran.clear();


        let keys = event_pump.keyboard_state();
        let lshift = if keys.is_scancode_pressed(Scancode::LShift) { true } else { false };
        let lctrl = if keys.is_scancode_pressed(Scancode::LCtrl) { true } else { false };
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. } => { is_running = false; },
                Event::MouseButtonDown { mouse_btn, x, y, .. } => {
                    match mouse_btn {
                        MouseButton::Left => {
                            for (i, rect) in list_rects.iter().enumerate() {
                                if rect.contains_point((x, y)) {
                                    if lshift {
                                        if !nodes_list[i].obstacle {
                                            node_start_mut_ptr = &mut nodes_list[i];
                                        }
                                    } else if lctrl {
                                        if !nodes_list[i].obstacle {
                                            node_end_ptr = &nodes_list[i];
                                        }
                                    } else {
                                        nodes_list[i].obstacle = !nodes_list[i].obstacle;
                                    }
                                    break;
                                }
                            }


                            for node in nodes_list.iter_mut() {
                                node.visited = false;
                                node.global_goal = f64::INFINITY;
                                node.local_goal = f64::INFINITY;
                                node.parent = None;
                            }

                            solve_astar(node_start_mut_ptr, node_end_ptr);
                        },

                        _ => {},
                    }
                },
                _ => {}
            }
        }

        core.ekran.set_draw_color((255, 0, 0));
        for node in nodes_list.iter() {
            for neighbour in node.neighbours.iter() {
                core.ekran.draw_line(node.point, neighbour.point).unwrap();
            }
        }

        for (i, rect) in list_rects.iter().enumerate() {
            let anode = &nodes_list[i];

            if anode.obstacle {
                core.ekran.set_draw_color((104, 161, 150));
            } else if node_start_mut_ptr as *const SNode == anode {
                core.ekran.set_draw_color((255, 255, 50));
            } else if node_end_ptr == anode {
                core.ekran.set_draw_color((0, 0, 255));
            } else if anode.visited {
                core.ekran.set_draw_color((125, 0, 0));
            } else {
                core.ekran.set_draw_color((255, 0, 0));
            }
            core.ekran.fill_rect(*rect).unwrap();
        }

        {
            let mut p: _ = Some(node_end_ptr);
            core.ekran.set_draw_color((255, 255, 255));

            while let Some(pa) = p {
                unsafe {
                    if let Some(papa) = (*pa).parent {
                        core.ekran.draw_line((*pa).point, (*papa).point).unwrap();
                    }

                    p = (*pa).parent;
                }
            }
        }


        core.ekran.present();
        core.fps_ctrl.end();
    }
}
