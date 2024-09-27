// For dht11 sensor
#[cfg(feature = "dht11")]
use dht_sensor::dht11::Reading;

#[cfg(feature = "dht22")]
use dht_sensor::dht22::Reading;

pub enum Value {
    Dht11Value(i8, u8),
    Dht22Value(f32, f32),
}

pub fn get(read: Reading) -> Value {
    let (temp, humi) = (read.temperature, read.relative_humidity);
    cfg_if::cfg_if! {
       if #[cfg(feature = "dht11")] {
       return Value::Dht11Value(temp, humi)

     } else if #[cfg(feature = "dht22")] {
       return Value::Dht22Value(temp, humi)
     }
    }
}

impl Value {
    #[cfg(feature = "dht11")]
    pub fn value(self) -> (i8, u8) {
        if let Value::Dht11Value(temp, humi) = self {
            return (temp, humi);
        } else {
            return (0, 0);
        }
    }

    #[cfg(feature = "dht22")]
    pub fn value(self) -> (f32, f32) {
        if let Value::Dht22Value(temp, humi) = self {
            return (temp, humi);
        } else {
            return (0.0, 0.0);
        }
    }
}
