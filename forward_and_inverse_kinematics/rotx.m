function mat = rotx(theta)
mat = [
    1 0 0;
    0 cos(theta) -sin(theta);
    0 sin(theta) cos(theta)
];
end