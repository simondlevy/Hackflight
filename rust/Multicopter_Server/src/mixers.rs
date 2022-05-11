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
}
