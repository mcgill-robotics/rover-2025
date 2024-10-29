import { ReactNode } from "react";

interface ContainerProps {
    title: string;
    width: string;
    height: string;
    border?: string;
    children?: ReactNode;
}

export function Container({ title, width, height, border, children }: ContainerProps) {
    return (
        <div style={{ height, width, border }}>
            <h1>{title}</h1>

            {children}

        </div>
    );
}



