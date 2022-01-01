#![allow(non_snake_case)]

use std::collections::{ VecDeque};
use sdl2::render::WindowCanvas;
use sdl2::{Sdl, TimerSubsystem};
use sdl2::event::Event;
use sdl2::keyboard::Scancode;
use sdl2::mouse::MouseButton;
use sdl2::rect::{Point, Rect};

pub struct Renderer {
    pub sdl_context: Sdl,
    pub ekran: WindowCanvas,
    // pub ekran_rect: Rect,
    pub fps_ctrl: FpsCapDeltaTime,
    // pub display_resolution: Vec<DisplayMode>,
}

impl Renderer {
    pub fn new(title: &str) -> Renderer {
        // let (width, height, flag, fps, monitor) = load_resolution_info();

        // init systems.
        let sdl_context = sdl2::init().unwrap();
        let video_subsystem = sdl_context.video().unwrap();


/*
        let video_displays = video_subsystem.num_video_displays().unwrap();

        let (dis_bound, monitor) = match video_subsystem.display_bounds(monitor) {
            Ok(dis) => (dis, monitor),
            Err(e) => {
                log::WARN!("Numer of monitors Err: {}", e);
                (video_subsystem.display_bounds(0).unwrap(), 0)
            },
        };
        log::TEST!("{:?}", dis_bound);
        log::TEST!("number of video_displays: {}", video_displays);

        let mode_n = video_subsystem.num_display_modes(monitor).unwrap();

        let mut display_resolution = vec![];

        for i in 0..mode_n {
            match video_subsystem.display_mode(0, i) {
                Ok(dsp) => {
                    display_resolution.push(dsp);
                }
                Err(e) => {
                    log::WARN!("Error {}", e);
                }
            }
        }
*/

        // create a window.
        let mut win = video_subsystem.window(title, 1280, 720);
        let window = win.position_centered()
                .build()
                .unwrap();



        // get the canvas
        let mut ekran = window.into_canvas().build().unwrap();
        ekran.set_logical_size(1280, 720).unwrap();


        Renderer {
            ekran,
            // ekran_rect: Rect::new(0, 0, width, height),
            fps_ctrl: FpsCapDeltaTime::new(sdl_context.timer().unwrap(), 60),
            sdl_context,
            // display_resolution,
        }
    }
}

pub struct FpsCapDeltaTime{
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

    pub fn start(&mut self){
        self.cap_frame_start = self.ttime.ticks();

        self.fps_last_time = self.fps_now;
        self.fps_now = self.ttime.performance_counter();

        self.dt = ((self.fps_now - self.fps_last_time) * 1000) as f64 / self.ttime.performance_frequency() as f64;

    }

    pub fn end(&mut self){
        self.cap_frame_end = self.ttime.ticks() - self.cap_frame_start;
        if self.cap_frame_end < self.frame_delay {
            self.ttime.delay(self.frame_delay - self.cap_frame_end);
        }
    }
}

struct SNode<'node>{
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


    pub fn heuristic(&self, other: &SNode) -> f64 {
        return self.distance(other);
    }
}

fn solve_astar(node_start: *mut SNode, node_end: &SNode){

    let mut curen_node = node_start;
    unsafe {
        (*curen_node).local_goal = 0.0;
        (*curen_node).global_goal = (*curen_node).heuristic(node_end);
    }


    let mut list_not_tested_nodes = VecDeque::new();
    list_not_tested_nodes.push_back(curen_node);

    while ! list_not_tested_nodes.is_empty() {
        unsafe {
            list_not_tested_nodes.make_contiguous().sort_by(|lhs, rhs| { (*(*lhs)).global_goal.partial_cmp(&(*(*rhs)).global_goal).unwrap() });

            while ! list_not_tested_nodes.is_empty() && (*(*list_not_tested_nodes.front().unwrap())).visited {
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

                let possibly_lower_goal = (*curen_node).local_goal + (*curen_node).distance(neighbour);
                if possibly_lower_goal < neighbour.local_goal {
                    neighbour.parent = Some(curen_node);
                    neighbour.local_goal = possibly_lower_goal;
                    neighbour.global_goal = neighbour.local_goal + neighbour.heuristic(node_end);
                }
            }
        }
    }
}



/// https://www.youtube.com/watch?v=icZj67PTFhc&list=WL&index=4&t=526s
fn main() {
    const MAP_WIDHT: usize = 20;
    const MAP_HEIGHT: usize = 20;
    const START_OFFSET: i32 = 1;

    let mut core = Renderer::new("a start pathing");


    let mut event_pump = core.sdl_context.event_pump().unwrap();


    let mut list_rects = vec![];
    let mut nodes_list = vec![];


    for x in START_OFFSET..MAP_WIDHT as i32 + START_OFFSET  {
        for y in START_OFFSET..MAP_HEIGHT as i32 + START_OFFSET  {
            let pos_rect = Rect::new(x * 30, y * 30, 20, 20);
            list_rects.push(pos_rect);
            nodes_list.push(SNode{
                obstacle: false,
                visited: false,
                global_goal: f64::MAX,
                local_goal: f64::MAX,
                point: pos_rect.center(),
                neighbours: vec![],
                parent: None
            })
        }
    }
    {
        let node_list_ptr: *mut _ = &mut nodes_list;

        for x in 0..MAP_WIDHT {
            for y in 0..MAP_HEIGHT {
                unsafe {
                    if y > 0 {
                        nodes_list[y * MAP_WIDHT + x].neighbours.push(&mut (*node_list_ptr)[(y - 1) * MAP_WIDHT + (x + 0)]);
                    }
                    if y < MAP_HEIGHT - 1 {
                        nodes_list[y * MAP_WIDHT + x].neighbours.push(&mut (*node_list_ptr)[(y + 1) * MAP_WIDHT + (x + 0)]);
                    }
                    if x > 0 {
                        nodes_list[y * MAP_WIDHT + x].neighbours.push(&mut (*node_list_ptr)[(y + 0) * MAP_WIDHT + (x - 1)]);
                    }
                    if x < MAP_WIDHT - 1 {
                        nodes_list[y * MAP_WIDHT + x].neighbours.push(&mut (*node_list_ptr)[(y + 0) * MAP_WIDHT + (x + 1)]);
                    }
                }
            }
        }
    }
    let mut node_start: *const _ = &nodes_list[2 * MAP_WIDHT + 10];
    let mut node_start_mut: *mut _ = &mut nodes_list[2 * MAP_WIDHT + 10];
    let mut node_end: *const _ = &nodes_list[17 * MAP_WIDHT + 10];

    let mut running = true;

//--------- LOOP
    while running {
        core.fps_ctrl.start();
        core.ekran.set_draw_color((0, 0, 0));
        core.ekran.clear();



        let keys = event_pump.keyboard_state();
        let lshift = if keys.is_scancode_pressed(Scancode::LShift){ true } else { false };
        let lctrl = if keys.is_scancode_pressed(Scancode::LCtrl){ true } else { false };
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. } => { running = false; },
                Event::MouseButtonDown { mouse_btn, x, y , .. } => {
                    match mouse_btn {
                        MouseButton::Left => {

                            for (i, rect) in list_rects.iter().enumerate() {
                                if rect.contains_point((x, y)) {
                                    if lshift {
                                        if !nodes_list[i].obstacle {
                                            node_start = &nodes_list[i];
                                            node_start_mut = &mut nodes_list[i];
                                        }
                                    } else if lctrl {
                                        if !nodes_list[i].obstacle {
                                            node_end = &nodes_list[i];
                                        }
                                    } else {
                                        nodes_list[i].obstacle = !nodes_list[i].obstacle;
                                    }
                                    break;
                                }
                            }


                            for node in nodes_list.iter_mut() {
                                node.visited = false;
                                node.global_goal = f64::MAX;
                                node.local_goal = f64::MAX;
                                node.parent = None;
                            }

                            solve_astar(node_start_mut, &nodes_list[17 * MAP_WIDHT + 10]);
                        },
                        // MouseButton::Right => {
                        //     for node in nodes_list.iter_mut() {
                        //         node.visited = false;
                        //         node.global_goal = f64::MAX;
                        //         node.local_goal = f64::MAX;
                        //         node.parent = None;
                        //     }
                        //
                        //     println!("solve");
                        //     solve_astar(node_start_mut, &nodes_list[17 * MAP_WIDHT + 10]);
                        // }

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
            } else if node_start == anode {
                core.ekran.set_draw_color((255, 255, 50));
            } else if node_end == anode {
                core.ekran.set_draw_color((0, 0, 255));
            } else {
                core.ekran.set_draw_color((255, 0, 0));
            }
            core.ekran.fill_rect(*rect).unwrap();
        }

        {
            let mut p = Some(node_end);
            core.ekran.set_draw_color((255, 255, 255));

            while let Some(pa) = p  {
                unsafe {
                    if let Some(papa) = (*pa).parent{

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
