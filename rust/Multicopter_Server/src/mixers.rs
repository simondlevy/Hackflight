pub struct Mixer {
    d: ((i8, i8, i8, i8),
        (i8, i8, i8, i8),
        (i8, i8, i8, i8),
        (i8, i8, i8, i8))
}

impl Mixer {
    pub fn new(d:((i8, i8, i8, i8),
                  (i8, i8, i8, i8),
                  (i8, i8, i8, i8),
                  (i8, i8, i8, i8))) -> Mixer{
        Mixer{d}
    }

    pub fn get_motors(self, u: (f32, f32, f32, f32)) -> f32 {
            1.0
        }

    // implementing as functions b/c no inheritence
    pub fn new_phantom_mixer() -> Mixer {
        Mixer::new(((1, -1, -1, 1), // 1 right front
                    (1, 1, 1, 1),   // 2 left rear
                    (1, 1, -1, -1), // 3 left front
                    (1, -1, 1, -1))) // 4 right rear
    }

    pub fn new_ingenuity_mixer() -> Mixer {
        Mixer::new(((1, 0, 0, 1), // 1 right front
                    (1, 0, 0, -1),// 2 left rear
                    (0, 1, 0, 0), // 3 left front
                    (0, 0, 1, 0))) // 4 right rear
    }
}
