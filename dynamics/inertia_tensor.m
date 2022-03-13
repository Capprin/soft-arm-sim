% computes local inertia tensor for cylindrical link at CoM
function I = inertia_tensor(length, mass, radius)
    % get principal moments about ea. axis at CoM
    Ixx = 1/2*mass*radius^2;
    Iyy = 1/12*mass*(3*radius^2 + length^2);
    Izz = Iyy;
    % compute tensor
    I = [Izz 0 0; 0 Iyy 0; 0 0 Ixx];
end