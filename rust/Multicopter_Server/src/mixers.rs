pub struct Mixer {
    d: [(i8, i8, i8, i8); 4]
}

impl Mixer {
    pub fn new(demands:[(i8, i8, i8, i8); 4]) -> Mixer {
        Mixer{d: demands}
    }
}

impl Mixer {
    pub fn get_motors(self, u: (f32, f32, f32, f32)) -> [f32; 4] {
        let mut omega = [0.0, 0.0, 0.0, 0.0];
        for i in 0..3 {
            omega[i] = u.0 * self.d[i].0 as f32 + u.1 * self.d[i].1 as f32 +
                      u.2 * self.d[i].2 as f32 + u.3 * self.d[i].3 as f32
        }
        return omega;
    }
}

impl Mixer {
    // implementing as functions b/c no inheritence
    pub fn new_phantom_mixer() -> Mixer {
        Mixer::new([(1, -1, -1, 1), // 1 right front
                    (1, 1, 1, 1),   // 2 left rear
                    (1, 1, -1, -1), // 3 left front
                    (1, -1, 1, -1)]) // 4 right rear
    }

    pub fn new_ingenuity_mixer() -> Mixer {
        Mixer::new([(1, 0, 0, 1), // 1 right front
             (1, 0, 0, -1),// 2 left rear
             (0, 1, 0, 0), // 3 left front
             (0, 0, 1, 0)]) // 4 right rear
    }
}
