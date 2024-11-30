import React, { useState, useEffect } from 'react';

const joints = ['waist', 'shoulder', 'elbow', 'wrist']


// allows to move the angle at the selected joint
export function ArmVisuals(){
    return (
        <div>

        </div>
    );
}

// allow to see which joint we are using
export function JointChoice(){
    const [Curr_index, setCurr_joint] = useState(0);
    useEffect(() => {
            const handleKey = (event: KeyboardEvent) => {
                setCurr_joint((index) => {
                    if (event.key === "ArrowUp"){
                        return index < 4-1 ? index+1 : 0;
                    }
                    else if (event.key === "ArrowDown"){
                        return index > 0 ? index-1 : 4-1;
                        
                    }
                    return index;

                });
                
            };

            window.addEventListener('keydown', handleKey)

            return () => {
                window.removeEventListener('keydown', handleKey);
            };
        },
    []);

    return(
        <div>
            <h2>
                Press up/down to cycle joints (waist, shoulder, elbow, joint)
            </h2>
            <h3>
                Current Joint: {joints[Curr_index]}
            </h3>
        </div>
    );
}



