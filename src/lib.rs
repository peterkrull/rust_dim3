#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}

#[allow(dead_code)]
#[derive(Debug)]
pub struct Quaternion {
    w:f32,
    x:f32,
    y:f32,
    z:f32,
}

#[allow(dead_code)]
#[allow(unused_variables)]
impl Quaternion {

    pub fn new (w:f32 , x:f32 , y:f32 , z:f32) -> Self {
        Self {w,x,y,z}
    }

    pub fn get_product(&self,q:Quaternion) -> Quaternion {
        Quaternion{
           w: self.w*q.w - self.x*q.x - self.y*q.y - self.z*q.z,  // new w
           x: self.w*q.x + self.x*q.w + self.y*q.z - self.z*q.y,  // new x
           y: self.w*q.y - self.x*q.z + self.y*q.w + self.z*q.x,  // new y
           z: self.w*q.z + self.x*q.y - self.y*q.x + self.z*q.w
        }
    }

    pub fn get_conjugate(&self) -> Quaternion {
        Quaternion{ w : self.w, x : -self.x, y : -self.y, z : -self.z }
    }

    pub fn get_magnitude(&self) -> f32 {
        (self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z).sqrt()
    }

    pub fn normalize(&mut self) {
        let norm = self.get_normalize();
        self.w = norm.w;
        self.x = norm.x;
        self.y = norm.y;
        self.z = norm.z;
    }

    pub fn get_normalize(&self) -> Quaternion {
        let mag = self.get_magnitude();
        Quaternion { 
            w : self.w / mag, 
            x : self.x / mag, 
            y : self.y / mag, 
            z : self.z / mag 
        }
    }

    pub fn from_to_rotation (v1 : &VectorFloat , v2 : &VectorFloat) -> Quaternion {
        let cross = v1.cross(v2);
        Quaternion { 
            w: ( v1.magnitude().powi(2) * v2.magnitude().powi(2) ).sqrt() + v1.dot(v2), 
            x: cross.x, 
            y: cross.y, 
            z: cross.z }.get_normalize()
    }
}



#[allow(dead_code)]
#[derive(Debug)]
pub struct VectorFloat {
    x:f32,
    y:f32,
    z:f32,
}

#[allow(dead_code)]
#[allow(unused_variables)]
impl VectorFloat {

    pub fn new (x:f32 , y:f32 , z:f32) -> Self {
        Self {x,y,z}
    }

    /// Adds `self` with `vec` and returns the result.
    pub fn add(&self,vec : VectorFloat) -> VectorFloat {
        VectorFloat {
            x : self.x + vec.x,
            y : self.y + vec.y,
            z : self.z + vec.z,
        }
    }

    /// Subtracts `vec` from `self` and returns the result.
    pub fn sub(&self,vec : VectorFloat) -> VectorFloat {
        VectorFloat {
            x : self.x - vec.x,
            y : self.y - vec.y,
            z : self.z - vec.z,
        }
    }
    
    /// Calculates the cross product between `self` and `vec` and returns the result.
    pub fn cross (&self,v :&VectorFloat) -> VectorFloat {
        VectorFloat { 
            x: self.z*v.y - self.y*v.z, 
            y: self.z*v.x - self.x*v.z, 
            z: self.x*v.y - self.y*v.x 
        }
    }

    pub fn prod(&self,num : f32) -> VectorFloat {
        VectorFloat {
            x : self.x * num,
            y : self.y * num,
            z : self.z * num,
        }
    }

    pub fn dot (&self,v :&VectorFloat) -> f32 {
        self.x * v.x + self.y * v.y + self.z * v.z
    }

    pub fn div(&self,v : f32) -> VectorFloat {
        VectorFloat {
            x : self.x / v,
            y : self.y / v,
            z : self.z / v,
        }
    }

    pub fn magnitude(&self) -> f32 {
        (self.x*self.x + self.y*self.y + self.z*self.z).sqrt()
    }

    pub fn get_normalized(&self) -> VectorFloat {
        self.div(self.magnitude())
    }

    pub fn rotate(&mut self,q:Quaternion) {
        let mut p = Quaternion { w : 0.0,x : self.x, y : self.y, z : self.z };

        p = q.get_product(p);
        p = p.get_product(q.get_conjugate());

        self.x = p.x;
        self.y = p.y;
        self.z = p.z;
    }

    pub fn quaternion_to (&self, v : &VectorFloat) -> Quaternion {
        let cross = self.cross(v);
        Quaternion { 
            w: ( self.magnitude().powi(2) * v.magnitude().powi(2) ).sqrt() + self.dot(v), 
            x: cross.x, 
            y: cross.y, 
            z: cross.z }.get_normalize()
    }

    pub fn get_rotated(&self , q:Quaternion) -> VectorFloat{
        let mut p = Quaternion { w : 0.0, x : self.x, y : self.y, z : self.z };

        p = q.get_product(p);

        p = p.get_product(q.get_conjugate());

        VectorFloat{ x : p.x, y : p.y, z : p.z }
    }
}


#[allow(dead_code)]
#[derive(Debug)]
pub struct Matrix3 {
    a : [[f32;3];3]
}

#[allow(dead_code)]
#[allow(unused_variables)]
impl Matrix3 {

    pub fn new(a : [[f32;3];3]) -> Self{
        Self {a}
    }

    pub fn new_from_rows(row1:[f32;3],row2:[f32;3],row3:[f32;3]) -> Matrix3{
        Matrix3::new([
            row1,
            row2,
            row3]
        )
    }

    pub fn new_from_cols(col1:[f32;3],col2:[f32;3],col3:[f32;3]) -> Matrix3{
        Matrix3::new([
            [col1[0],col2[0],col3[0]],
            [col1[1],col2[1],col3[1]],
            [col1[2],col2[2],col3[2]]]
        )
    }

    /// Returns a new matrix from a vector consisting of the diagonal entries.
    ///
    /// ``` no_run
    /// let m = Matrix3::new_from_diag(
    ///    [1.0 , 5.0 , 9.0]);
    /// ```
    /// Which is equivalent to :
    /// ``` no_run
    /// let m = Matrix3::new_from_rows(
    ///    [1.0 , 0.0 , 0.0],
    ///    [0.0 , 5.0 , 0.0],
    ///    [0.0 , 0.0 , 9.0]);
    /// ```
    pub fn new_from_diag(diag:[f32;3]) -> Matrix3{
        Matrix3::new([
            [diag[0],0.0,0.0],
            [0.0,diag[1],0.0],
            [0.0,0.0,diag[2]]]
        )
    }

    pub fn new_identity() -> Matrix3 {
        Matrix3::new_from_diag([1.0,1.0,1.0])
    }

    pub fn get_raw(&self) -> [[f32;3];3]{
        self.a
    }

    /// Transposes the `self` matrix and returns the result.
    ///
    /// ``` no_run
    /// let m = Matrix3::new_from_rows(
    ///    [1.0 , 2.0 , 3.0],
    ///    [4.0 , 5.0 , 6.0],
    ///    [7.0 , 8.0 , 9.0]);
    /// 
    /// let m_trans = m.transpose(); 
    /// ```
    pub fn transpose(&self) -> Matrix3 {
        Matrix3::new([
            [self.a[0][0],self.a[1][0],self.a[2][0]],
            [self.a[0][1],self.a[1][1],self.a[2][1]],
            [self.a[0][2],self.a[1][2],self.a[2][2]]]
        ) 
    }

    /// Calculates the matrix product between `self` and some matrix `m`.
    ///
    /// ``` no_run
    /// let m1 = Matrix3::new_from_rows(
    ///    [1.0 , 2.0 , 3.0],
    ///    [4.0 , 5.0 , 6.0],
    ///    [7.0 , 8.0 , 9.0]);
    /// 
    /// let m2 = Matrix3::new_from_rows(
    ///    [2.0 , 3.0 , 4.0],
    ///    [5.0 , 6.0 , 7.0],
    ///    [8.0 , 9.0 , 10.0]);
    /// 
    /// let prod = m1.product(m2); 
    /// ```
    pub fn product(&self,m:&Matrix3) -> Matrix3 {
        Matrix3::new([
            [self.a[0][0]*m.a[0][0] + self.a[0][1]*m.a[1][0] + self.a[0][2]*m.a[2][0] , self.a[0][0]*m.a[0][1] + self.a[0][1]*m.a[1][1] + self.a[0][2]*m.a[2][1]  , self.a[0][0]*m.a[0][2] + self.a[0][1]*m.a[1][2] + self.a[0][2]*m.a[2][2] ], 
            [self.a[1][0]*m.a[0][0] + self.a[1][1]*m.a[1][0] + self.a[1][2]*m.a[2][0] , self.a[1][0]*m.a[0][1] + self.a[1][1]*m.a[1][1] + self.a[1][2]*m.a[2][1]  , self.a[1][0]*m.a[0][2] + self.a[1][1]*m.a[1][2] + self.a[1][2]*m.a[2][2] ], 
            [self.a[2][0]*m.a[0][0] + self.a[2][1]*m.a[1][0] + self.a[2][2]*m.a[2][0] , self.a[2][0]*m.a[0][1] + self.a[2][1]*m.a[1][1] + self.a[2][2]*m.a[2][1]  , self.a[2][0]*m.a[0][2] + self.a[2][1]*m.a[1][2] + self.a[2][2]*m.a[2][2] ]]
        )
    }

    pub fn vector_product(&self,v:&VectorFloat) -> Matrix3 {
        Matrix3::new([
            [self.a[0][0]*v.x , self.a[0][1]*v.y , self.a[0][2]*v.z ] , 
            [self.a[1][0]*v.x , self.a[1][1]*v.y , self.a[1][2]*v.z ] , 
            [self.a[2][0]*v.x , self.a[2][1]*v.y , self.a[2][2]*v.z ] ]
        )
    }
}