import { createContext, PropsWithChildren, ReactElement, useContext } from 'react';
import useRoverROS, { RoverServices, UseRoverROSResult } from '../hooks/useRoverROS';
import { RoverTelemetry } from '../types/ros';

export interface RoverContextValue extends UseRoverROSResult {}

const RoverContext = createContext<RoverContextValue | null>(null);

export function RoverProvider({ children }: PropsWithChildren): ReactElement {
  const rover = useRoverROS();
  return <RoverContext.Provider value={rover}>{children}</RoverContext.Provider>;
}

export function useRover(): RoverContextValue {
  const ctx = useContext(RoverContext);
  if (!ctx) {
    throw new Error('useRover must be used within a RoverProvider');
  }
  return ctx;
}

export type { RoverTelemetry, RoverServices };
